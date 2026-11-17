# Two-wheel car: L9110S driver + Raspberry Pi Pico (MicroPython)
# Wiring (measured with bringup b.pins(); pins and inverts live in config.py):
#   GP0 -> A-1A   GP1 -> A-1B   (left wheel)
#   GP2 -> B-1A   GP3 -> B-1B   (right wheel)
#   Pico GND (pin 18) -> ground rail -> L9110S GND and battery (-)
#   Battery (+) (4x AA) -> switch -> L9110S VCC

import time
import config
from motor import Motor

DRIVE_SPEED = 0.3
SPIN_SPEED = 0.15


class Car:
    def __init__(self, left_pins=None, right_pins=None):
        lp = left_pins or config.LEFT_PINS
        rp = right_pins or config.RIGHT_PINS
        self.left = Motor(lp[0], lp[1], config.LEFT_INVERT, config.LEFT_DEADBAND, config.PWM_FREQ)
        self.right = Motor(rp[0], rp[1], config.RIGHT_INVERT, config.RIGHT_DEADBAND, config.PWM_FREQ)

    def stop(self, secs=0):
        """Stop both motors, then wait secs seconds."""
        self.left.stop()
        self.right.stop()
        if secs:
            time.sleep(secs)

    def drive(self, l, r, secs=None):
        """Set wheel speeds (-1 to 1). With secs, run that long and stop; otherwise keep running."""
        self.left.set(l)
        self.right.set(r)
        if secs is not None:
            time.sleep(secs)
            self.stop()

    def forward(self, secs=None, speed=DRIVE_SPEED):
        self.drive(speed, speed, secs)

    def backward(self, secs=None, speed=DRIVE_SPEED):
        self.drive(-speed, -speed, secs)

    def spin_left(self, secs=None, speed=SPIN_SPEED):
        self.drive(-speed, speed, secs)

    def spin_right(self, secs=None, speed=SPIN_SPEED):
        self.drive(speed, -speed, secs)
