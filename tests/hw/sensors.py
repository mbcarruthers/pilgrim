# Line sensor check: raw A0 readings as a fraction of 3.3 V. Motors stay off.
#   make hw-sensors
# Slide each HW-870 over the tape: its number should change by 0.3 or more.
# Ctrl-C to stop.

import time
import config
from linesensor import LineSensors

s = LineSensors(config.IR_LEFT_PIN, config.IR_RIGHT_PIN)
while True:
    a, b = s.raw()
    print("left (GP{}) {:.2f}    right (GP{}) {:.2f}".format(config.IR_LEFT_PIN, a, config.IR_RIGHT_PIN, b))
    time.sleep_ms(200)
