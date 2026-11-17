# Two HW-870 (TCRT5000) line sensors read through their A0 outputs (MicroPython, RP2040)
#   left HW-870 A0 -> GP26 (ADC0)     right HW-870 A0 -> GP28 (ADC2)
#   both VCC -> 3V3(OUT), both GND -> GND

import json
from machine import ADC, Pin
from follower import darkness

SAMPLES = 4      # ADC readings averaged per sample


class LineSensors:
    def __init__(self, left_pin, right_pin):
        self.left = ADC(Pin(left_pin))
        self.right = ADC(Pin(right_pin))
        self.swapped = False
        self.levels = None       # ((white, black) left, (white, black) right), raw 0-1

    def swap(self):
        """Exchange left and right (for sensors wired the other way round)."""
        self.left, self.right = self.right, self.left
        self.swapped = not self.swapped

    def raw_u16(self):
        """(left, right) as ADC counts, 0-65535."""
        a = b = 0
        for _ in range(SAMPLES):
            a += self.left.read_u16()
            b += self.right.read_u16()
        return a // SAMPLES, b // SAMPLES

    def raw(self):
        """(left, right) as a fraction of 3.3 V."""
        a, b = self.raw_u16()
        return a / 65535, b / 65535

    def read(self):
        """(left, right) darkness: 0 over white, 1 over the line. Needs levels set.
        A reading darker than the calibrated black becomes the new black, so a sensor
        that only brushed the line during calibration corrects itself."""
        a, b = self.raw()
        la, lb = self.levels
        if (a - la[1]) * (la[1] - la[0]) > 0:
            la = (la[0], a)
            self.levels = (la, lb)
        if (b - lb[1]) * (lb[1] - lb[0]) > 0:
            lb = (lb[0], b)
            self.levels = (la, lb)
        return darkness(a, la[0], la[1]), darkness(b, lb[0], lb[1])

    def save(self, path, both_on_line):
        try:
            with open(path, "w") as f:
                json.dump({"levels": self.levels, "swapped": self.swapped,
                           "both_on_line": both_on_line}, f)
        except OSError:
            pass

    def load(self, path):
        """Restore the last saved calibration. Returns its both_on_line flag, or None if there is none."""
        try:
            with open(path) as f:
                d = json.load(f)
            la, lb = d["levels"]
            if bool(d["swapped"]) != self.swapped:
                self.swap()
            self.levels = ((la[0], la[1]), (lb[0], lb[1]))
            return bool(d["both_on_line"])
        except (OSError, ValueError, KeyError, TypeError):
            return None
