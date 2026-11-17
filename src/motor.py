# L9110S motor channel driven by two PWM pins (MicroPython, RP2040)

from machine import Pin, PWM

FREQ = 1000      # PWM frequency, 1 kHz
MAX = 65535      # duty_u16 full scale
ZERO = 0.02      # commands smaller than this count as stop


class Motor:
    def __init__(self, pin_a, pin_b, invert=False, deadband=0.0, freq=FREQ):
        self.a = PWM(Pin(pin_a))
        self.b = PWM(Pin(pin_b))
        self.a.freq(freq)
        self.b.freq(freq)
        self.invert = invert       # True if this wheel spins the wrong way
        self.deadband = deadband   # lowest duty that turns the wheel (bringup b.deadband())
        self.stop()

    def set(self, speed):
        """speed: -1.0 (full reverse) .. 0 (stop) .. 1.0 (full forward).
        Any non-zero speed starts at the deadband, so small commands still turn the wheel."""
        speed = max(-1.0, min(1.0, speed))
        if abs(speed) < ZERO:
            self.stop()
            return
        if self.invert:
            speed = -speed
        duty = int((self.deadband + (1 - self.deadband) * abs(speed)) * MAX)
        if speed > 0:            # 1A = PWM, 1B = LOW
            self.b.duty_u16(0)
            self.a.duty_u16(duty)
        else:                    # 1A = LOW, 1B = PWM
            self.a.duty_u16(0)
            self.b.duty_u16(duty)

    def stop(self):              # both inputs LOW
        self.a.duty_u16(0)
        self.b.duty_u16(0)
