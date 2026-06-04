#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Servo ID scanner - read-only, no movement.
Run this to find which servo IDs are alive and their current positions.
"""

import time
from servo import LobotServo

# ---------- Config ----------
SERIAL_PORT = "/dev/ttyUSB0"

# IDs to scan. Lobot servo IDs are 0-253 (254 = broadcast, can't read).
# Put all IDs you want to check here.
SCAN_IDS = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

# ---------- Scan ----------
if __name__ == "__main__":
    found = {}
    dead = {}

    print("Serial port: {}".format(SERIAL_PORT))
    print("Scanning servo IDs: {}".format(SCAN_IDS))
    print("-" * 40)

    for sid in SCAN_IDS:
        try:
            servo = LobotServo(SERIAL_PORT, sid, timeout=0.2)
            pos = servo.read_position()
            servo.close()

            if pos is not None:
                print("  ID={:3d}  [OK]    position = {}".format(sid, pos))
                found[sid] = pos
            else:
                print("  ID={:3d}  [--]    no response".format(sid))
                dead[sid] = None

        except Exception as e:
            print("  ID={:3d}  [ERR]   {}".format(sid, e))
            dead[sid] = None

        time.sleep(0.05)

    # ---------- Summary ----------
    print("-" * 40)
    print("Scan done. {} servo(s) found.".format(len(found)))
    for sid, pos in found.items():
        print("  ID={}  position={}".format(sid, pos))

    if len(found) == 0:
        print("")
        print("No servos found! Check:")
        print("  1. Is the servo control board powered? (12V connected?)")
        print("  2. Is the serial port correct? (current: {})".format(SERIAL_PORT))
        print("  3. Is the USB cable plugged in?")
        print("  4. Try expanding scan range: SCAN_IDS = range(0, 254)")


# ============================================================
# Everything below is commented out (movement tests).
# Uncomment when you've identified your servo IDs.
# ============================================================

# ACTIVE_SERVO_IDS = [1, 2, 4, 5]  # <- put the IDs you found here
#
# def move_servo(servo_id, position, time_ms=1000):
#     position = max(0, min(1000, position))
#     servo = LobotServo(SERIAL_PORT, servo_id)
#     servo.move(position, time_ms)
#     print("Servo ID={} -> position {}".format(servo_id, position))
#     servo.close()
#
# def read_all_positions():
#     for sid in ACTIVE_SERVO_IDS:
#         servo = LobotServo(SERIAL_PORT, sid)
#         pos = servo.read_position()
#         servo.close()
#         if pos is not None:
#             print("Servo ID={} current position: {}".format(sid, pos))
#         else:
#             print("Servo ID={} read failed".format(sid))
