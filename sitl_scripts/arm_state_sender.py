#!/usr/bin/env python3
"""
Transport: MAVLink DEBUG_FLOAT_ARRAY (ID 350), array_id = ARM_STATE_ARRAY_ID.
Layout in data[]:  [0:4] angle  [4:8] velocity  [8:12] acceleration
"""
import os
os.environ["MAVLINK20"] = "1"      # Nachricht-ID 350 > 255 -> MAVLink 2 zwingend (vor dem pymavlink-Import!)

import time
from pymavlink import mavutil

ARM_STATE_ARRAY_ID = 42      # muss mit der Firmware uebereinstimmen
NUM_JOINTS = 4


class ArmStateSender:
    def __init__(self, connection: str, baud: int = 115200):
        mavutil.set_dialect("development")   # vor mavlink_connection(); PX4 baut gegen diesen Dialekt
        self._m = mavutil.mavlink_connection(
            connection, baud=baud, source_system=1, source_component=191  # 191 = onboard computer
        )
        self._t0 = time.monotonic()

    def send(self, q, qd, qdd, tau):
        """q, qd, qdd, tau: Erwartet Liste [4, 4, 4, 3]"""
        data = [0.0] * 58
        data[0:4] = [float(x) for x in q]
        data[4:8] = [float(x) for x in qd]
        data[8:12] = [float(x) for x in qdd]
        data[12:15] = [float(x) for x in tau]
        t_us = int((time.monotonic() - self._t0) * 1e6)
        self._m.mav.debug_float_array_send(t_us, b"arm_state", ARM_STATE_ARRAY_ID, data)
