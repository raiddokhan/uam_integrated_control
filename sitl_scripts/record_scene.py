#!/usr/bin/env python3
"""Start PX4 SITL (headless), run scene.py, record it, and collect the results.

Output in <out-dir>/<timestamp>/:  raw.mp4, overlay.mp4, <flight>.ulg

  python3 record_scene.py
"""
from __future__ import annotations
import argparse
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from gz.transport13 import Node                          # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V                  # noqa: E402
from gz.msgs10.entity_factory_pb2 import EntityFactory   # noqa: E402
from gz.msgs10.boolean_pb2 import Boolean                # noqa: E402

PX4_DIR = Path("/home/raid/PX4-Hardware")
WORLD = "default"
MODEL = "tarot_arm"
CAP = HERE / "_cam_capture.py"
CAM_MODEL = "spectator_cam"
CAM_TOPIC = "/spectator/image"

# Camera placement (relative to the model spawn point)
CAM_DIST = 6.5        # m, distance to spawn point
CAM_HEIGHT = 2.8      # m, height above ground
CAM_SHIFT = 0         # m, lateral offset (camera still aims at the drone)
CAM_PAN_RIGHT = 1.0   # m, pure translation to the right
CAM_TARGET_Z = 2.0    # m, height the camera aims at
CAM_AZIMUTH = 0.0     # deg, 0 = camera at -Y looking towards +Y
CAM_HFOV = 0.95       # rad


# --------------------------------------------------------------------------- #
# Simulator (PX4 SITL + Gazebo)
# --------------------------------------------------------------------------- #
class Sim:
    def __init__(self, px4_dir: Path):
        self.dir = px4_dir
        self.rootfs = px4_dir / "build/px4_sitl_default/rootfs"
        self.proc = None
        self.t_start = 0.0
        self.ready = threading.Event()
        self.tail: deque[str] = deque(maxlen=40)

    def _pump(self):
        for line in self.proc.stdout:
            self.tail.append(line.rstrip())
            if "Ready for takeoff" in line:
                self.ready.set()

    def start(self, timeout: float = 300.0):
        self.t_start = time.time()
        self.proc = subprocess.Popen(
            ["make", "px4_sitl", f"gz_{MODEL}"], cwd=self.dir,
            env={**os.environ, "HEADLESS": "1"},
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            start_new_session=True)
        threading.Thread(target=self._pump, daemon=True).start()
        t_end = time.time() + timeout
        while not self.ready.wait(1.0):
            if self.proc.poll() is not None or time.time() > t_end:
                raise RuntimeError("PX4 SITL did not become ready:\n" + "\n".join(self.tail))
        print("[rec] PX4 SITL ready")

    def stop(self):
        if self.proc is None:
            return
        try:
            os.killpg(self.proc.pid, signal.SIGINT)
            self.proc.wait(timeout=20)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            pass
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def latest_log(self) -> Path | None:
        logs = [p for p in (self.rootfs / "log").rglob("*.ulg") if p.stat().st_mtime >= self.t_start]
        return max(logs, key=lambda p: p.stat().st_mtime) if logs else None


# --------------------------------------------------------------------------- #
# Gazebo: pose tracking, camera
# --------------------------------------------------------------------------- #
class PoseTracker:
    def __init__(self, node: Node, topic: str, model: str):
        self.model, self.name, self.sim_t = model, None, None
        self._rows: list[tuple] = []
        self._lock = threading.Lock()
        if not node.subscribe(Pose_V, topic, self._cb):
            raise RuntimeError(f"could not subscribe {topic}")

    def _cb(self, msg: Pose_V):
        t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        self.sim_t = t
        if self.name is None:
            cands = [p.name for p in msg.pose if p.name.startswith(self.model)]
            if cands:
                self.name = min(cands, key=len)
        for p in msg.pose:
            if p.name == self.name:
                with self._lock:
                    self._rows.append((t, p.position.x, p.position.y, p.position.z))
                break

    def array(self) -> np.ndarray:
        with self._lock:
            return np.array(self._rows, float).reshape(-1, 4)

    def last_pos(self):
        with self._lock:
            return np.array(self._rows[-1][1:4]) if self._rows else None


def spawn_camera(node: Node, pos, yaw, pitch, w, h, fps, outdir: Path):
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{CAM_MODEL}">
    <static>true</static>
    <pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} 0 {pitch:.5f} {yaw:.5f}</pose>
    <link name="link">
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{fps}</update_rate>
        <topic>{CAM_TOPIC}</topic>
        <camera>
          <horizontal_fov>{CAM_HFOV}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>100</far></clip>
        </camera>
      </sensor>
    </link>
  </model>
</sdf>"""
    svc = f"/world/{WORLD}/create"
    req = EntityFactory()
    req.sdf = sdf
    ok, rep = node.request(svc, req, EntityFactory, Boolean, 15000)

    if ok and getattr(rep, 'data', False):
        return

    sdf_file = outdir / "spectator_cam.sdf"
    sdf_file.write_text(sdf)
    r = subprocess.run(
        ["gz", "service", "-s", svc, "--reqtype", "gz.msgs.EntityFactory",
         "--reptype", "gz.msgs.Boolean", "--timeout", "15000",
         "--req", f'sdf_filename: "{sdf_file}"'],
        capture_output=True, text=True)

    if r.returncode != 0 or "true" not in r.stdout:
        raise RuntimeError(f"camera spawn failed via CLI. Error: {r.stderr.strip()}")


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


class Cam:
    """Pinhole model of the Gazebo camera (looks along +X, Y left, Z up)."""

    def __init__(self, pos, yaw, pitch, w, h):
        self.t, self.w, self.h = np.asarray(pos, float), w, h
        self.R = rot_rpy(0.0, pitch, yaw)
        self.f = (w / 2) / math.tan(CAM_HFOV / 2)

    def project(self, P) -> np.ndarray:
        pc = (np.atleast_2d(P).astype(float) - self.t) @ self.R
        x = np.where(pc[:, 0] > 0.05, pc[:, 0], np.nan)
        return np.stack([self.w / 2 - pc[:, 1] / x * self.f,
                         self.h / 2 - pc[:, 2] / x * self.f], axis=1)


def place_camera(p0, w, h) -> tuple[Cam, np.ndarray, float, float]:
    a = math.radians(CAM_AZIMUTH)
    pos = np.array([p0[0] + CAM_DIST * math.sin(a), p0[1] - CAM_DIST * math.cos(a), CAM_HEIGHT])
    d = np.array([p0[0] - pos[0], p0[1] - pos[1]])
    d /= np.linalg.norm(d)
    pos[:2] += CAM_SHIFT * np.array([d[1], -d[0]])
    yaw = math.atan2(p0[1] - pos[1], p0[0] - pos[0])
    pitch = math.atan2(CAM_HEIGHT - CAM_TARGET_Z, math.hypot(p0[0] - pos[0], p0[1] - pos[1]))
    pos[:2] += CAM_PAN_RIGHT * np.array([math.sin(yaw), -math.cos(yaw)])
    return Cam(pos, yaw, pitch, w, h), pos, yaw, pitch


# --------------------------------------------------------------------------- #
# Video capture (_cam_capture.py subprocess)
# --------------------------------------------------------------------------- #
def start_capture(raw: Path, fps: int) -> subprocess.Popen:
    return subprocess.Popen(
        [sys.executable, str(CAP), "--topic", CAM_TOPIC, "--out", str(raw),
         "--fps", str(fps), "--max-seconds", "900"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


def stop_capture(cap: subprocess.Popen):
    cap.send_signal(signal.SIGINT)
    try:
        out = cap.communicate(timeout=180)[0]
        print("[rec] capture:", next((l for l in out.splitlines() if l.startswith("{")), "?"))
    except Exception:                                    # noqa: BLE001
        cap.kill()


# --------------------------------------------------------------------------- #
# Scene
# --------------------------------------------------------------------------- #
def run_scene():
    import scene as S

    print("0. Connect to PX4")
    S.connect_px4()
    S.arm_sender = S.ArmStateSender(S.ARM_STATE_CONN)

    print("1. Start pose")
    S.start_pose(S.HOME)

    print("3. Takeoff (Position Controller an)")
    S.takeoff()
    S.hold(20)

    print("4. Setze Parameter (Failsafes & Kaskade)")
    S.configure_px4_parameters(enable_ff=True)
    S.hold(5)

    print("5. Perform trajectory")
    S.move(S.HOME, S.DOWN, duration=5)
    S.hold(30)
    S.move(S.DOWN, S.STRAIGHT, duration=5)
    S.hold(30)

    print("6. Reset Parameters")
    S.reset_px4_parameters()
    S.hold(5)

    print("5. Land")
    S.land()
    S.hold(10)

# --------------------------------------------------------------------------- #
# Overlay video
# --------------------------------------------------------------------------- #
def make_overlay(raw: Path, out: Path, poses: np.ndarray, cam: Cam, fps: int):
    import imageio.v2 as imageio
    import scene as S
    from PIL import Image, ImageDraw, ImageFont

    ftimes = np.load(str(raw) + ".times.npy")
    pt, pp = poses[:, 0], poses[:, 1:4]
    t_start = pt[0]

    # Fester Ursprung: x = 0, y = 0, z = TAKEOFF_HEIGHT
    ref = np.array([-0.017, -0.23, 2.85])
    dev = pp - ref
    peak = float(np.hypot(dev[:, 0], dev[:, 1]).max()) if len(dev) > 0 else 0.0
    ext = max(0.10, 1.2 * peak)

    ref_px = cam.project(ref)[0]
    uv_all = cam.project(pp)
    W, H = cam.w, cam.h
    s = W / 960.0
    try:
        f_s = ImageFont.truetype("DejaVuSans.ttf", int(16 * s))
    except Exception:                                    # noqa: BLE001
        f_s = ImageFont.load_default()
    RED, GRN, ORG, WHT = (230, 60, 50, 255), (40, 200, 70, 255), (255, 170, 40, 255), (255, 255, 255, 255)

    rd = imageio.get_reader(raw)
    wr = imageio.get_writer(out, fps=fps, codec="libx264", quality=8, macro_block_size=None)
    for i, fr in enumerate(rd):
        t = float(ftimes[min(i, len(ftimes) - 1)])
        im = Image.fromarray(fr).convert("RGBA")
        ov = Image.new("RGBA", im.size, (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)

        P = np.array([np.interp(t, pt, pp[:, k]) for k in range(3)])
        dx, dy, dz = (P - ref) * 100.0                   # cm, ENU
        dxy = math.hypot(dx, dy)
        cur = cam.project(P)[0]
        seg = (pt <= t)
        pk = float(np.hypot(dev[seg, 0], dev[seg, 1]).max() * 100) if seg.any() else 0.0

        # Origin crosshair am festen Punkt (0, 0, TAKEOFF_HEIGHT)
        if not np.isnan(ref_px).any():
            u, v = ref_px
            d.line([(u, 0), (u, H)], fill=(40, 200, 70, 90), width=1)
            d.line([(0, v), (W, v)], fill=(40, 200, 70, 90), width=1)
            d.line([(u - 14 * s, v), (u + 14 * s, v)], fill=GRN, width=int(3 * s))
            d.line([(u, v - 14 * s), (u, v + 14 * s)], fill=GRN, width=int(3 * s))

        # Trail
        idx = np.where(seg)[0][::3]
        trail = [tuple(uv_all[j]) for j in idx if not np.isnan(uv_all[j]).any()]
        if len(trail) > 1:
            d.line(trail, fill=ORG, width=int(2 * s))

        # Deviation arrow
        if not (np.isnan(ref_px).any() or np.isnan(cur).any()) and math.hypot(dxy, dz) > 1.0:
            (u0, v0), (u1, v1) = ref_px, cur
            d.line([(u0, v0), (u1, v1)], fill=RED, width=int(3 * s))
            a = math.atan2(v1 - v0, u1 - u0)
            for da in (2.6, -2.6):
                d.line([(u1, v1), (u1 - 12 * s * math.cos(a + da), v1 - 12 * s * math.sin(a + da))],
                       fill=RED, width=int(3 * s))

        # Readout unten
        d.rectangle([0, H - 30 * s, 560 * s, H], fill=(0, 0, 0, 190))
        d.text((10 * s, H - 26 * s),
               f"dx {dx:+5.0f}  dy {dy:+5.0f}  dz {dz:+5.0f} cm   |dxy| {dxy:4.0f}  peak {pk:4.0f} cm",
               font=f_s, fill=(255, 130, 120, 255))
        d.text((W - 110 * s, H - 26 * s), f"t = {t - t_start:+6.1f} s", font=f_s, fill=WHT)

        # Top-view inset (ENU: x right, y up)
        ins = int(0.21 * W)
        x0, y0 = W - ins - 12 * s, 44 * s
        cx, cy, k = x0 + ins / 2, y0 + ins / 2, (ins / 2 * 0.9) / ext
        d.rectangle([x0, y0, x0 + ins, y0 + ins], fill=(0, 0, 0, 160), outline=(255, 255, 255, 200))
        d.line([(x0, cy), (x0 + ins, cy)], fill=(255, 255, 255, 70))
        d.line([(cx, y0), (cx, y0 + ins)], fill=(255, 255, 255, 70))
        pts = [(cx + ex * k, cy - ey * k) for ex, ey in dev[idx, :2]]
        if len(pts) > 1:
            d.line(pts, fill=ORG, width=max(1, int(2 * s)))
        px_, py_, r = cx + dx / 100 * k, cy - dy / 100 * k, 5 * s
        d.ellipse([px_ - r, py_ - r, px_ + r, py_ + r], fill=RED)
        d.text((x0 + 4 * s, y0 + 2 * s), f"top view +-{ext * 100:.0f} cm", font=f_s, fill=WHT)

        wr.append_data(np.asarray(Image.alpha_composite(im, ov).convert("RGB")))
    rd.close()
    wr.close()


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--px4-dir", default=str(PX4_DIR))
    ap.add_argument("--out-dir", default=str(HERE / "recordings"))
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    outdir = Path(args.out_dir) / time.strftime("%Y%m%d_%H%M%S")
    outdir.mkdir(parents=True, exist_ok=True)
    raw, final = outdir / "raw.mp4", outdir / "overlay.mp4"
    print(f"[rec] output: {outdir}")

    sim = Sim(Path(args.px4_dir))
    poses, cam = None, None
    success = False
    try:
        sim.start()

        node = Node()
        trk = PoseTracker(node, f"/world/{WORLD}/dynamic_pose/info", MODEL)
        t_end = time.time() + 20
        while (trk.last_pos() is None or trk.sim_t is None) and time.time() < t_end:
            time.sleep(0.2)
        if trk.last_pos() is None:
            raise RuntimeError(f"no pose received for model '{MODEL}*'")

        cam, cpos, yaw, pitch = place_camera(trk.last_pos(), args.width, args.height)
        spawn_camera(node, cpos, yaw, pitch, args.width, args.height, args.fps, outdir)
        time.sleep(2.0)

        cap = start_capture(raw, args.fps)
        time.sleep(2.0)
        try:
            run_scene()
            success = True
        finally:
            stop_capture(cap)
        poses = trk.array()
    finally:
        sim.stop()

    ulg = sim.latest_log()
    if ulg:
        shutil.copy2(ulg, outdir / ulg.name)
        print(f"[rec] log: {ulg.name}")
    else:
        print("[rec] WARNING: no .ulg found")

    if success and raw.exists() and poses is not None and len(poses) > 0:
        print("[rec] rendering overlay ...")
        make_overlay(raw, final, poses, cam, args.fps)
    else:
        print("[rec] run incomplete -> no overlay")
    print(f"[rec] done: {outdir}")


if __name__ == "__main__":
    main()
