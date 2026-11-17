# HC-SR04 ultrasonic distance sensor (MicroPython, RP2040)
#   GP19 -> Trig     GP18 <- Echo (through the 1:2 divider)

from machine import Pin, time_pulse_us
import time

TIMEOUT_US = 30000    # echo wait, about 5 m range
CM_PER_US = 0.0343    # speed of sound


class Ultrasonic:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT, value=0)
        self.echo = Pin(echo_pin, Pin.IN, Pin.PULL_DOWN)   # stays LOW if unplugged or unpowered

    def ping_us(self, timeout_us=TIMEOUT_US):
        """Send one pulse and return the echo time in us (negative on timeout)."""
        self.trig.value(0)
        time.sleep_us(5)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        return time_pulse_us(self.echo, 1, timeout_us)

    def ping_cm(self, timeout_us=6000):
        """One quick reading for use inside a control loop: distance in cm, or None for no echo.
        Blocks for at most about twice timeout_us."""
        if self.echo.value():        # the previous echo is still coming in
            return None
        t = self.ping_us(timeout_us)
        if t <= 0:
            return None
        return t * CM_PER_US / 2

    def distance(self, samples=5):
        """Return the median distance in cm, or None if no echo came back. Slow: about 60 ms per sample."""
        readings = []
        for _ in range(samples):
            t = self.ping_us()
            if t > 0:
                readings.append(t)
            time.sleep_ms(60)
        if not readings:
            return None
        readings.sort()
        return readings[len(readings) // 2] * CM_PER_US / 2
