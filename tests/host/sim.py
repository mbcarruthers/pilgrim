# A small physics simulation of the robot, so the real src/ code can run on a PC.
# It fakes MicroPython's `machine`, `rp2` and the time functions, and models the
# two wheels, the two line sensors over a tape track, and the HC-SR04.

import math
import os
import random
import sys
import time
import types

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src")
sys.path.insert(0, SRC)

WORLD = None


class SimDone(KeyboardInterrupt):
    """Raised when the simulated time runs out (like Ctrl-C on the robot)."""


# ---------- tracks ----------

class Stadium:
    """Two straights of length L joined by half circles of radius R. Bottom straight is y = 0."""

    def __init__(self, L=0.8, R=0.25):
        self.L, self.R = L, R

    def dist(self, x, y):
        cx = min(max(x, 0.0), self.L)
        return abs(math.hypot(x - cx, y - self.R) - self.R)


class Polyline:
    """A closed path of straight segments (sharp corners)."""

    def __init__(self, pts):
        self.segs = [(pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]

    def dist(self, x, y):
        best = 1e9
        for (ax, ay), (bx, by) in self.segs:
            dx, dy = bx - ax, by - ay
            t = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)
            t = min(max(t, 0.0), 1.0)
            best = min(best, math.hypot(x - ax - t * dx, y - ay - t * dy))
        return best


class Both:
    """Two tracks laid on the same floor (for crossings)."""

    def __init__(self, one, two):
        self.one, self.two = one, two

    def dist(self, x, y):
        return min(self.one.dist(x, y), self.two.dist(x, y))


# ---------- the world ----------

class World:
    def __init__(self, track, x=0.2, y=0.0, heading=0.0, tape_mm=19.0, sensors_swapped=False,
                 black_reads_high=True, end_ms=30000, seed=1,
                 true_deadband=(0.40, 0.32), vmax=(0.55, 0.62)):
        self.track = track
        self.x, self.y, self.th = x, y, heading
        self.tape = tape_mm / 1000.0
        self.sensors_swapped = sensors_swapped
        self.white, self.black = (0.15, 0.80) if black_reads_high else (0.85, 0.20)
        self.end_ms = end_ms
        self.rng = random.Random(seed)
        self.true_deadband, self.vmax = true_deadband, vmax
        self.t = 0.0
        self.vl = self.vr = 0.0
        self.duty = {}
        self.led = 0
        self.button = lambda t: 0
        self.obstacle = lambda t: None       # distance in cm, or None
        self.events = []                     # (time_ms, function(world))
        self.trace = []                      # (t, lateral error of the sensor midpoint in m, moving)
        self.travel = 0.0
        # geometry from the photos and bringup: sensors ahead of the axle, spacing, wheel track
        self.ahead, self.half_gap, self.wheel_track = 0.11, 0.0075, 0.127

    def sensor_xy(self, side):               # side +1 = left, -1 = right, 0 = midpoint
        s = side * self.half_gap
        return (self.x + self.ahead * math.cos(self.th) - s * math.sin(self.th),
                self.y + self.ahead * math.sin(self.th) + s * math.cos(self.th))

    def darkness(self, side):
        d = self.track.dist(*self.sensor_xy(side))
        blur = 0.003
        return min(1.0, max(0.0, (self.tape / 2 + blur - d) / (2 * blur)))

    def adc(self, pin):
        side = 1 if pin == 26 else -1
        if self.sensors_swapped:
            side = -side
        v = self.white + (self.black - self.white) * self.darkness(side)
        v += self.rng.uniform(-0.01, 0.01)
        return int(min(1.0, max(0.0, v)) * 65535)

    def _wheel(self, fwd_pin, rev_pin, i):
        def g(pin):
            d = self.duty.get(pin, 0) / 65535
            return max(0.0, (d - self.true_deadband[i]) / (1 - self.true_deadband[i]))
        return self.vmax[i] * (g(fwd_pin) - g(rev_pin))

    def advance(self, ms):
        while ms > 0:
            h = min(1.0, ms)
            ms -= h
            while self.events and self.events[0][0] <= self.t:
                self.events.pop(0)[1](self)
            dt = h / 1000.0
            k = dt / 0.08                    # motors take about 80 ms to change speed
            self.vl += (self._wheel(1, 0, 0) - self.vl) * k
            self.vr += (self._wheel(3, 2, 1) - self.vr) * k
            v = (self.vl + self.vr) / 2
            self.th += (self.vr - self.vl) / self.wheel_track * dt
            self.x += v * math.cos(self.th) * dt
            self.y += v * math.sin(self.th) * dt
            self.travel += abs(v) * dt
            self.t += h
            self.trace.append((self.t, self.track.dist(*self.sensor_xy(0)),
                               abs(self.vl) + abs(self.vr) > 0.01))
            if self.t >= self.end_ms:
                raise SimDone()

    def motors_off(self):
        return all(self.duty.get(p, 0) == 0 for p in (0, 1, 2, 3))


# ---------- fake MicroPython modules ----------

class Pin:
    IN, OUT, PULL_UP, PULL_DOWN = 0, 1, 2, 3

    def __init__(self, n, mode=None, pull=None, value=None):
        self.n = n

    def value(self, v=None):
        if v is None:
            return 0
        if self.n == "LED":
            WORLD.led = 1 if v else 0

    def on(self):
        self.value(1)

    def off(self):
        self.value(0)


class PWM:
    def __init__(self, pin):
        self.n = pin.n

    def freq(self, f):
        pass

    def duty_u16(self, d):
        WORLD.duty[self.n] = d


class ADC:
    def __init__(self, pin):
        self.n = pin.n

    def read_u16(self):
        return WORLD.adc(self.n)


def time_pulse_us(pin, level, timeout_us):
    cm = WORLD.obstacle(WORLD.t)
    us = None if cm is None else cm * 2 / 0.0343
    if us is None or us > timeout_us:
        WORLD.advance(timeout_us / 1000.0)
        return -2 if us is None else -1
    WORLD.advance(us / 1000.0)
    return int(us)


def install():
    machine = types.ModuleType("machine")
    machine.Pin, machine.PWM, machine.ADC, machine.time_pulse_us = Pin, PWM, ADC, time_pulse_us
    sys.modules["machine"] = machine
    rp2 = types.ModuleType("rp2")
    rp2.bootsel_button = lambda: WORLD.button(WORLD.t)
    sys.modules["rp2"] = rp2
    time.ticks_ms = lambda: int(WORLD.t)
    time.ticks_diff = lambda a, b: a - b
    time.ticks_add = lambda a, b: a + b
    time.sleep_ms = lambda ms: WORLD.advance(ms)
    time.sleep_us = lambda us: WORLD.advance(us / 1000.0)
    time.sleep = lambda s: WORLD.advance(s * 1000.0)


def run(world, **overrides):
    """Run the real Robot in this world until its time is up. Returns the robot."""
    global WORLD
    WORLD = world
    install()
    import importlib
    import config
    importlib.reload(config)
    config.CAL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_cal_test.json")
    for k, v in overrides.items():
        setattr(config, k, v)
    if not overrides.get("KEEP_CAL") and os.path.exists(config.CAL_FILE):
        os.remove(config.CAL_FILE)
    import robot
    r = robot.Robot()
    try:
        r.run()
    except SimDone:
        pass
    world.robot = r
    return r
