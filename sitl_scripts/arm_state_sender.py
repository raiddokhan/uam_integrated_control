#!/usr/bin/env python3
"""
Transport: MAVLink DEBUG_FLOAT_ARRAY (ID 350).

Senden (Python -> PX4), array_id = ARM_STATE_ARRAY_ID:
    data[0:4] angle  data[4:8] velocity  data[8:12] acceleration  data[12:15] tau

Empfangen (PX4 -> Python), array_id = TORQUE_DBG_ARRAY_ID:
    data[0]   torque_setpoint_y (float32 Skalar aus vehicle_torque_setpoint)
"""
import os
os.environ["MAVLINK20"] = "1"      # Nachricht-ID 350 > 255 -> MAVLink 2 zwingend (vor dem pymavlink-Import!)

import threading
import time
from pymavlink import mavutil

ARM_STATE_ARRAY_ID = 42      # Python -> PX4, muss mit der Firmware uebereinstimmen
TORQUE_DBG_ARRAY_ID = 43     # PX4 -> Python, muss zum MAVLink-Stream in der Firmware passen
NUM_JOINTS = 4


class ArmStateSender:
    def __init__(self, connection: str, baud: int = 115200):
        mavutil.set_dialect("development")   # vor mavlink_connection(); PX4 baut gegen diesen Dialekt
        self._m = mavutil.mavlink_connection(
            connection, baud=baud, source_system=1, source_component=191  # 191 = onboard computer
        )

        if connection.startswith("udpout:"):
            self._m.port.bind(("0.0.0.0", 14541))

        self._t0 = time.monotonic()

        # Empfangene Werte (geschuetzt durch _rx_lock)
        self._rx_lock = threading.Lock()
        self._torque_setpoint_y = 0.0
        self._last_rx_time = None            # monotonic-Zeitpunkt des letzten gueltigen Pakets

        # Event-Objekt für die ereignisbasierte Signalisierung neuer Daten
        self.new_data_event = threading.Event()

        # Separater Lock fuer das Senden
        self._tx_lock = threading.Lock()

        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    # ------------------------------------------------------------------ Senden
    def send(self, q, qd, qdd, tau, dis_y):
        """q, qd, qdd, tau: Erwartet Liste [4, 4, 4, 3]"""
        data = [0.0] * 58
        data[0:4] = [float(x) for x in q]
        data[4:8] = [float(x) for x in qd]
        data[8:12] = [float(x) for x in qdd]
        data[12:15] = [float(x) for x in tau]
        data[16] = dis_y
        t_us = int((time.monotonic() - self._t0) * 1e6)
        with self._tx_lock:
            self._m.mav.debug_float_array_send(t_us, b"arm_state", ARM_STATE_ARRAY_ID, data)

    # --------------------------------------------------------------- Empfangen
    def _rx_loop(self):
        while self._running:
            try:
                msg = self._m.recv_match(type="DEBUG_FLOAT_ARRAY", blocking=True, timeout=0.1)
            except Exception as e:           # z.B. serieller Fehler, Thread nicht sterben lassen
                print(f"[ArmStateSender] RX-Fehler: {e}")
                time.sleep(0.1)
                continue

            if msg is None or msg.array_id != TORQUE_DBG_ARRAY_ID:
                continue

            with self._rx_lock:
                self._torque_setpoint_y = float(msg.data[0])
                self._last_rx_time = time.monotonic()

            # Signalisiere dem Kontroll-Thread, dass neue Daten bereitstehen
            self.new_data_event.set()

    @property
    def stab_joint_angle(self) -> float:
        with self._rx_lock:
            return self._torque_setpoint_y

    def rx_age(self):
        """Sekunden seit dem letzten empfangenen Paket (None, falls noch keines kam)."""
        with self._rx_lock:
            if self._last_rx_time is None:
                return None
            return time.monotonic() - self._last_rx_time

    # ------------------------------------------------------------------ Aufraeumen
    def close(self):
        self._running = False
        # Event einmal setzen, falls der RX-Thread gerade in einem Timeout hängt
        self.new_data_event.set()
        self._rx_thread.join(timeout=1.0)
        self._m.close()


if __name__ == "__main__":
    sender = ArmStateSender("udpout:127.0.0.1:14580")
    try:
        while True:
            sender.send([0.0] * 4, [0.0] * 4, [0.0] * 4, [0.0] * 3)
            age = sender.rx_age()
            print(f"Vehicle_Y_Torque={sender._torque_setpoint_y:.4f}  rx_age={age}")
            time.sleep(0.02)                    # 50 Hz
    except KeyboardInterrupt:
        sender.close()
