# Lab 4: autonomous line follower (Raspberry Pi Pico, L9110S, 2x HW-870, HC-SR04)
#
# Put the robot on the line and power it on. After 2 s it sweeps its sensors across
# the line to calibrate, then follows it. All settings are in config.py.

import gc
from robot import Robot

gc.collect()
try:    # collect garbage early and often, so the Pico's small heap doesn't fragment
    gc.threshold(gc.mem_free() // 4 + gc.mem_alloc())
except AttributeError:
    pass

Robot().run()
