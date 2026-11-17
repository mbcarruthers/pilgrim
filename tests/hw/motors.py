# Motor check. Lift the robot or give it room, battery switch on.
#   make hw-motors
# Expect: forward, backward, spin left (anticlockwise seen from above), spin right.
# Wrong? Fix LEFT_PINS / RIGHT_PINS / *_INVERT in src/config.py.

import time
from car import Car

car = Car()
try:
    for name, l, r in (("forward", 0.3, 0.3), ("backward", -0.3, -0.3),
                       ("spin left", -0.15, 0.15), ("spin right", 0.15, -0.15)):
        print(name)
        car.drive(l, r, 1)
        time.sleep(1)
    print("done")
finally:
    car.stop()
