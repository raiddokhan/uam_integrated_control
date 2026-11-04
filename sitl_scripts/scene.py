#!/usr/bin/env python3
import math
import numpy as np
import time
import subprocess
import threading

from arm_state_sender import ArmStateSender

import gz.transport13 as gz_transport
import gz.msgs10.double_pb2 as gz_msg
from pymavlink import mavutil


# MANIPULATOR PROPERTIES
LIMITS = {"joint2": (-2.0, 2.0), "joint3": (-1.70, 1.70), "joint4": (-1.70, 1.97)}
JOINTS = ("joint1", "joint2", "joint3", "joint4")
TOPIC = "/omx/{}/cmd_pos"

VEL_MAX = 3.6   # rad/s REAL: Maximum 6.36 rad/s (safe)
ACC_MAX = 5.0   # rad/s²
Q2_MAX = 0.8    # rad (angle limit for stabilization)
FC_STAB = 0.5   # Hz
DEADZONE_TAU = 0.01

# DEFINITION of JOINT POSES
TAKEOFF = dict(joint1=0.0, joint2=-0.6, joint3=1.2, joint4=-0.6)
DOWN = dict(joint1=0.0, joint2=0.0, joint3=-2.2, joint4=0.0)
HOME = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0)
STRAIGHT = dict(joint1=0.0, joint2=1.8, joint3=-2.2, joint4=0.0)
SWING_POS = dict(joint1=0.0, joint2=0.8, joint3=-2.2, joint4=0.0)
SWING_NEG = dict(joint1=0.0, joint2=-0.8, joint3=-2.2, joint4=0.0)

PX4_BIN = "/home/raid/PX4-Hardware/build/px4_sitl_default/bin"
STREAM_RATE = 250          # Hz, arm_state an PX4 (waehrend der Trajektorie)
ARM_STATE_CONN = "udpout:127.0.0.1:14580"

TAKEOFF_HEIGHT = 2.5   # m

mav = None
arm_sender = None

node = gz_transport.Node()
pubs = {j: node.advertise(TOPIC.format(j), gz_msg.Double) for j in JOINTS}
time.sleep(0.3)

# DEFINITION of THREADs
_stab_thread = None
_stab_stop = threading.Event()


class ArmAngleFilter:
    """Tiefpass 1. Ordnung -> Rate Limiter -> Positionslimit (Winkelraum).
    Der begrenzte Winkel ist der Filterzustand (Anti-Windup fuer Rate und Position)."""
    def __init__(self, fc=FC_STAB, q_min=-Q2_MAX, q_max=Q2_MAX, vel_max=VEL_MAX):
        self.tau_f = 1.0 / (2.0 * math.pi * fc)
        self.q_min, self.q_max = q_min, q_max
        self.vel_max = vel_max
        self.q = 0.0
        self.omega = 0.0
        self.initialized = False

    def reset(self):
        self.initialized = False

    def update(self, q_cmd, dt):
        q_cmd = float(q_cmd)
        if not self.initialized:
            self.q = float(np.clip(q_cmd, self.q_min, self.q_max))
            self.omega = 0.0
            self.initialized = True
            return self.q

        # 1) Tiefpass auf dem tatsaechlich erreichten Winkel
        alpha = dt / (dt + self.tau_f)
        dq = alpha * (q_cmd - self.q)

        # 2) Rate Limiter: max. Aenderung pro Schritt
        dq_max = self.vel_max * dt
        dq = float(np.clip(dq, -dq_max, dq_max))

        # 3) Positionssaettigung (gesaettigter Wert wird Zustand)
        q_new = float(np.clip(self.q + dq, self.q_min, self.q_max))

        self.omega = (q_new - self.q) / dt
        self.q = q_new
        return self.q
# ------------------------------------------------------------ Regressoren
REGRESSORS = {
    # Trajektorie: tau_y = Phi(q)·theta, Phi = [sin q2, sin q3, cos q2, cos q3, 1]
    "trajectory": dict(type="trig",theta=[0.09659031, -0.01007421, -0.01441170, 0.05132045, 0.01060782]),
    "stabilizer": dict(type="linear", a=0.08097375, b=0.00629492)
}

def calc_feedforward(q, regressor="trajectory") -> float:
    """Feedforward-Drehmoment (Y) fuer die Gelenkstellung q mit dem gewaehlten Regressor."""
    reg = REGRESSORS[regressor]
    q = np.asarray(q, dtype=float)

    if reg["type"] == "trig":
        Phi = np.concatenate([np.sin(q[1:3]), np.cos(q[1:3]), [1.0]])
        return float(np.dot(Phi, reg["theta"]))
    if reg["type"] == "linear":
        return float(reg["a"] * q[1] + reg["b"])
    raise ValueError(f"Unbekannter Regressor-Typ: {reg['type']}")

def hover_torque(regressor="trajectory") -> float:
    return calc_feedforward([0, 0, 0, 0], regressor)

def torque_to_angle(tau, regressor="stabilizer") -> float:
    return tau / REGRESSORS[regressor]["a"]

# ---------------------------------------------------------------- PX4 helpers
def px4_cmd(*args):
    subprocess.run([f"{PX4_BIN}/px4-commander", *args], check=True)

def px4_param_set(name, value):
    subprocess.run([f"{PX4_BIN}/px4-param", "set", name, str(value)], check=True)

def configure_px4_parameters(enable_ff, enable_tdc, enable_arm_stab):
    print("Konfiguriere PX4 Parameter (Failsafes & Kaskade)...")

    # 1. RC-Checks und Failsafes deaktivieren
    px4_param_set("COM_RC_IN_MODE", 1)      # RC nicht zwingend erforderlich
    px4_param_set("NAV_RCL_ACT", 0)         # Keine Aktion bei RC-Verlust
    px4_param_set("COM_RCL_EXCEPT", 4)      # RC-Loss im Hold/Position Mode ignorieren

    # 2. Gewünschte Kaskadenwerte setzen
    px4_param_set("MPC_XY_P", 0.0)           # Kein Zurückholen zur Position
    px4_param_set("MPC_XY_VEL_I_ACC", 0.0)  # Kein Nachkriechen durch Integrator
    px4_param_set("MPC_XY_VEL_P_ACC", 2.0)  # Dämpfung P
    px4_param_set("MPC_XY_VEL_D_ACC", 0.2)  # Dämpfung D

    # 3. PID Ratenregler oder TDC Ratenregler
    if enable_tdc:
        px4_param_set("TDC_GBAR_R", 400.0)
        px4_param_set("TDC_GBAR_P", 400.0)
        px4_param_set("TDC_GBAR_Y", 200.0)

        px4_param_set("TDC_RT_MODE", 1.0)
    else:
        px4_param_set("TDC_RT_MODE", 0.0)

    # 4. FF Kompensation an oder aus
    if enable_ff:
        px4_param_set("ARM_FF_EN", 1.0)
    else:
        px4_param_set("ARM_FF_EN", 0.0)

    # 5. Manipulator Assisted Stabilization an oder aus
    if enable_arm_stab:
        px4_param_set("ARM_STAB_EN", 1.0)
    else:
        px4_param_set("ARM_STAB_EN", 0.0)

def reset_px4_parameters():
    print("PX4 Parameter zurücksetzen (Nur MPC_XY*)")
    px4_param_set("MPC_XY_P", 0.95)
    px4_param_set("MPC_XY_VEL_I_ACC", 4.0)
    px4_param_set("MPC_XY_VEL_P_ACC", 3.0)
    px4_param_set("MPC_XY_VEL_D_ACC", 0.2)

    px4_param_set("TDC_RT_MODE", 0.0)
    px4_param_set("ARM_FF_EN", 0.0)
    px4_param_set("ARM_STAB_EN", 0.0)

def takeoff():
    px4_cmd("takeoff")

def land():
    px4_cmd("land")

def connect_px4(url="udpin:0.0.0.0:14540"):
    global mav
    mav = mavutil.mavlink_connection(url)
    mav.wait_heartbeat()
    print(f"Verbunden mit System {mav.target_system}")

# ----------------------------------------------------------------- Manipulator
def min_duration(a, b, vel_max=VEL_MAX, acc_max=ACC_MAX):
    """Schnellstmögliche Dauer für das Cosinus-Profil in move()."""
    dq = max(abs(b[j] - a[j]) for j in JOINTS)
    t_vel = dq * math.pi / (2.0 * vel_max)
    t_acc = math.sqrt(dq * math.pi**2 / (2.0 * acc_max))
    return max(t_vel, t_acc)

def publish_pose(pose):
    for j in JOINTS:
        pubs[j].publish(gz_msg.Double(data=pose[j]))

def start_pose(pose_start):
    publish_pose(pose_start)

def move(a, b, duration=None, rate=50, stream_rate=STREAM_RATE, transmit=True):
    if duration is None:
        duration = min_duration(a, b)
    w = math.pi / duration
    dt = 1.0 / stream_rate
    gz_every = max(1, round(stream_rate / rate))

    t0 = time.monotonic()
    t_next = t0
    k = 0
    while True:
        t = time.monotonic() - t0
        done = t >= duration
        if done:
            f, fd, fdd = 1.0, 0.0, 0.0
        else:
            f = 0.5 * (1 - math.cos(w * t))
            fd = 0.5 * w * math.sin(w * t)
            fdd = 0.5 * w * w * math.cos(w * t)

        q = [a[j] + (b[j] - a[j]) * f for j in JOINTS]
        qd = [(b[j] - a[j]) * fd for j in JOINTS]
        qdd = [(b[j] - a[j]) * fdd for j in JOINTS]

        if k % gz_every == 0 or done:
            for j, qj in zip(JOINTS, q):
                pubs[j].publish(gz_msg.Double(data=qj))

        FF = calc_feedforward(q, "trajectory") - hover_torque("trajectory")

        if arm_sender is not None:
            if transmit:
                arm_sender.send(q, qd, qdd, [0, FF, 0], 0)

        if done:
            break
        k += 1
        t_next += dt
        time.sleep(max(0.0, t_next - time.monotonic()))

def hold(duration):
    time.sleep(duration)
# ----------------------------------------------------------------- Manipulator-assisted Disturbance Rejection (Idle Stabilizer)
def induce_disturbance(dis_y=-0.3, duration=0.5):
    if arm_sender is None:
        print("WARNUNG: arm_sender ist None")
        return
    q = [0, 0, 0, 0]; qd = [0, 0, 0, 0]; qdd = [0, 0, 0, 0]; tau = [0, 0, 0];
    arm_sender.send(q, qd, qdd, tau, dis_y)
    time.sleep(duration)
    arm_sender.send(q, qd, qdd, tau, 0)

def deadzone(x, dz=DEADZONE_TAU):
    """Weiche Totzone: 0 innerhalb von +-dz, darüber um dz verschoben."""
    if x > dz:
        return x - dz
    if x < -dz:
        return x + dz
    return 0.0

def stabilize_with_joint1(stop_event, max_rx_age=0.5, nominal_dt=1.0/100):
    lpf = ArmAngleFilter(fc=FC_STAB, q_min=-Q2_MAX, q_max=Q2_MAX, vel_max=VEL_MAX)
    hover = hover_torque("stabilizer")
    t_last = None

    while not stop_event.is_set():
        got_data = arm_sender.new_data_event.wait(timeout=max_rx_age)
        if stop_event.is_set():
            break
        arm_sender.new_data_event.clear()

        now = time.monotonic()
        dt = nominal_dt if t_last is None else min(max(now - t_last, 1e-3), 0.2)
        t_last = now

        age = arm_sender.rx_age()
        if not got_data or age is None or age > max_rx_age:
            tau_in = 0.0
        else:
            tau_in = deadzone(-1 * arm_sender._torque_setpoint_y)

        # Sollwinkel aus Rohmoment (Vorzeichen wie bisher), dann Filter + Limits
        angle_cmd = torque_to_angle(tau_in, "stabilizer")
        angle = lpf.update(angle_cmd, dt)       # erreichbarer Winkel
        omega = lpf.omega                       # konsistente Geschwindigkeit

        q   = [0.0, angle, -2.2, 0.0]
        qd  = [0.0, omega, 0.0, 0.0]
        qdd = [0.0] * 4
        FF  = calc_feedforward(q, "stabilizer") - hover   # Moment passend zum realisierten Winkel

        publish_pose(dict(joint1=q[0], joint2=q[1], joint3=q[2], joint4=q[3]))
        arm_sender.send(q, qd, qdd, [0, FF, 0], 0)

def start_stabilizer():
    global _stab_thread
    if arm_sender is None:
        print("WARNUNG: arm_sender ist None")
        return
    if _stab_thread is not None and _stab_thread.is_alive():
        print("Stabilizer laeuft bereits")
        return
    _stab_stop.clear()
    _stab_thread = threading.Thread(
        target=stabilize_with_joint1, args=(_stab_stop,), daemon=True
    )
    _stab_thread.start()

def stop_stabilizer():
    global _stab_thread
    _stab_stop.set()
    if _stab_thread is not None:
        _stab_thread.join(timeout=1.0)
        _stab_thread = None

from scipy.io import loadmat

def stream_stab_angles(mat_path, var_name="stab_angle"):
    rate = 50 # Vehicle_torque_setpoint updates with 50Hz

    angles = np.asarray(loadmat(mat_path)[var_name], dtype=float).flatten()
    print(f"{len(angles)} Samples geladen, Dauer: {len(angles) / rate:.2f} s")

    dt = 1.0 / rate
    t_next = time.monotonic()
    for angle in angles:
        publish_pose(dict(joint1=0.0, joint2=float(angle), joint3=-2.2, joint4=0.0))
        t_next += dt
        time.sleep(max(0.0, t_next - time.monotonic()))
