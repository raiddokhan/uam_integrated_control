#!/usr/bin/env python3
"""Subscribe to a gz camera image topic and record frames to a video.

Runs as a subprocess of render_scenario.py.  Buffers every frame with its sim
timestamp; on SIGINT/SIGTERM writes <out> (mp4 via imageio-ffmpeg).  Prints one
JSON line to stdout with n_frames and the sim-time span.

  python3 _cam_capture.py --topic /spectator/image --out clip.mp4 --fps 25
"""
from __future__ import annotations
import argparse
import json
import os
import signal
import sys
import time

import numpy as np

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

TIMES: list[float] = []
_writer = None
_stop = False


def _decode(msg: Image):
    h, w = msg.height, msg.width
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    pf = msg.pixel_format_type
    if pf in (3, 0):            # RGB_INT8 (0 = UNKNOWN, assume RGB)
        return buf.reshape(h, w, 3)
    if pf == 5:                 # BGR_INT8
        return buf.reshape(h, w, 3)[:, :, ::-1]
    if pf == 4:                 # RGBA_INT8
        return buf.reshape(h, w, 4)[:, :, :3]
    if pf in (1,):             # L_INT8
        return np.repeat(buf.reshape(h, w, 1), 3, axis=2)
    return buf.reshape(h, w, 3)


def _cb(msg: Image):
    if _stop or _writer is None:
        return
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    try:
        _writer.append_data(_decode(msg))           # stream to disk, don't buffer
        TIMES.append(t)
    except Exception as e:                                  # noqa: BLE001
        print(f"[cap] frame fail: {e!r}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/spectator/image")
    ap.add_argument("--out", required=True)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--max-seconds", type=float, default=120.0)
    a = ap.parse_args()

    import imageio.v2 as imageio
    global _writer, _stop
    os.makedirs(os.path.dirname(os.path.abspath(a.out)) or ".", exist_ok=True)
    _writer = imageio.get_writer(a.out, fps=a.fps, codec="libx264", quality=8,
                                 macro_block_size=None)

    node = Node()
    if not node.subscribe(Image, a.topic, _cb):
        sys.exit(f"[cap] could not subscribe {a.topic}")
    print(f"[cap] subscribed {a.topic}", file=sys.stderr)

    def _sig(*_):
        global _stop
        _stop = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    t0 = time.time()
    while not _stop and time.time() - t0 < a.max_seconds:
        time.sleep(0.05)

    _stop = True
    time.sleep(0.5)                       # let any in-flight callback finish
    try:
        _writer.close()
    except Exception:                                      # noqa: BLE001
        pass
    n = len(TIMES)
    span = (TIMES[-1] - TIMES[0]) if n > 1 else 0.0
    if n:
        np.save(a.out + ".times.npy", np.array(TIMES))
    print(f"[cap] {n} frames, sim span {span:.1f}s -> {a.out}", file=sys.stderr)
    print(json.dumps(dict(n_frames=n, sim_span=span,
                          t_first=TIMES[0] if n else None,
                          t_last=TIMES[-1] if n else None, out=a.out)))
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
