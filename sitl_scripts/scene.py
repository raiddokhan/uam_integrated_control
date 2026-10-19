#!/usr/bin/env python3
# Szenario: Takeoff -> Position Mode (angepasste Kaskade) -> Manipulator-Trajektorie -> Landung
import math
import numpy as np
import time
import subprocess

from arm_state_sender import ArmStateSender   # MUSS vor pymavlink importiert werden (setzt MAVLINK20)

import gz.transport13 as gz_transport
import gz.msgs10.double_pb2 as gz_msg
from pymavlink import mavutil


# MANIPULATOR PROPERTIES
LIMITS = {"joint2": (-2.0, 2.0), "joint3": (-1.70, 1.70), "joint4": (-1.70, 1.97)}
JOINTS = ("joint1", "joint2", "joint3", "joint4")
TOPIC = "/omx/{}/cmd_pos"

# DEFINITION of JOINT POSES
LIFTOFF = dict(joint1=0.0, joint2=-0.6, joint3=1.2, joint4=-0.6)
DOWN = dict(joint1=0.0, joint2=0, joint3=-1.7, joint4=0.0)
HOME = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0)
STRAIGHT = dict(joint1=0.0, joint2=1.8, joint3=-1.7, joint4=0.0)

PX4_BIN = "/home/raid/PX4-Hardware/build/px4_sitl_default/bin"
RATE = 100                 # Hz, Gazebo-Kommandos
STREAM_RATE = 250          # Hz, arm_state an PX4 (waehrend der Trajektorie)
ARM_STATE_CONN = "udpout:127.0.0.1:14580"

TAKEOFF_HEIGHT = 2.5   # m

mav = None
arm_sender = None

node = gz_transport.Node()
pubs = {j: node.advertise(TOPIC.format(j), gz_msg.Double) for j in JOINTS}
time.sleep(0.3)


# ---------------------------------------------------------------- PX4 helpers
def px4_cmd(*args):
    subprocess.run([f"{PX4_BIN}/px4-commander", *args], check=True)

def px4_param_set(name, value):
    subprocess.run([f"{PX4_BIN}/px4-param", "set", name, str(value)], check=True)

def configure_px4_parameters(enable_ff):
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

    # 3. PID Ratenregler und nicht TDC erstmal
    px4_param_set("TDC_RT_MODE", 0.0)

    if enable_ff:
        # 4. Activate FF Kompensation (Test)
        px4_param_set("ARM_FF_EN", 1.0)

def reset_px4_parameters():
    print("PX4 Parameter zurücksetzen (Nur MPC_XY*)")
    px4_param_set("MPC_XY_P", 0.95)
    px4_param_set("MPC_XY_VEL_I_ACC", 4.0)
    px4_param_set("MPC_XY_VEL_P_ACC", 3.0)
    px4_param_set("MPC_XY_VEL_D_ACC", 0.2)

    px4_param_set("ARM_FF_EN", 0.0)

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
def calc_feedforward(q) -> float:
	# Calculate Feedforward Torque in Y direction

    #theta = [-1.31398566, -1.49035542, -0.00521637, 0.12516566, -0.11922401] # Only HOME -> STRAIGHT

    theta = [0.09659031, -0.01007421, -0.01441170, 0.05132045, 0.01060782] # HOME -> DOWN -> STRAIGHT

    sin_term = np.sin(q[1:3])
    cos_term = np.cos(q[1:3])
    Phi = np.concatenate([sin_term, cos_term, [1]])

    ff_torque = np.dot(Phi, theta)

    return ff_torque

def publish_pose(pose):
    for j in JOINTS:
        pubs[j].publish(gz_msg.Double(data=pose[j]))

def start_pose(pose_start):
    publish_pose(pose_start)

def move(a, b, duration=2.0, rate=RATE, stream_rate=STREAM_RATE, transmit_ff=True):
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

        HOVER_TORQUE = calc_feedforward([0, 0, 0, 0])
        FF = calc_feedforward(q) - HOVER_TORQUE # Calculate the Feedforward Torque

        if arm_sender is not None:
            arm_sender.send(q, qd, qdd, [0, FF, 0])

        if done:
            break
        k += 1
        t_next += dt
        time.sleep(max(0.0, t_next - time.monotonic()))

def hold(duration):
    time.sleep(duration)
