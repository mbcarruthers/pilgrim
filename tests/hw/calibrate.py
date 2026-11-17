# Calibration check: runs the start-up sweep, then shows what the sensors see. No driving after.
#   make hw-calibrate
# Put the robot on the line first. It spins left, then right, and prints the result.
# Then slide it by hand: each sensor should read near 0.00 over white and 1.00 over the tape.
# Ctrl-C to stop.

import time
from robot import Robot

robot = Robot()
try:
    time.sleep(2)
    ok = robot.calibrate()
    robot.car.stop()
    while ok:
        l, r = robot.sensors.read()
        print("left {:.2f}   right {:.2f}   error {:+.2f}".format(l, r, l - r))
        time.sleep_ms(200)
finally:
    robot.car.stop()
    robot.led.off()
