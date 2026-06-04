# -*- coding: utf-8 -*-
from servo import LobotServo
import time

SERIAL = "/dev/ttyUSB0"

# -- ID=0: base rotation --
s0 = LobotServo(SERIAL, 0)
pos = s0.read_position()
print("Current pos:", pos)

# small move +50
s0.move(pos + 50, 1000)
time.sleep(1.2)
print("After move:", s0.read_position())

# move back
s0.move(pos, 1000)
time.sleep(1.2)
print("After return:", s0.read_position())
s0.close()
 