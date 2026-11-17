# The line-following robot: builds every part from config.py and runs the control loop.
#
# Onboard LED:  steady = following      slow blink = lost the line, or paused
#               fast blink = obstacle ahead, or calibration failed (press BOOTSEL to retry)
# BOOTSEL button: pause / resume.

import gc
import time
from machine import Pin
import config as C
from car import Car
from linesensor import LineSensors
from ultrasonic import Ultrasonic
from follower import Calibration, Follower, Obstacle

try:
    from rp2 import bootsel_button
except ImportError:
    def bootsel_button():
        return 0

SLOW_BLINK_MS = 500
FAST_BLINK_MS = 100


class Robot:
    def __init__(self):
        self.car = Car()
        self.led = Pin("LED", Pin.OUT)
        self.sensors = LineSensors(C.IR_LEFT_PIN, C.IR_RIGHT_PIN)
        self.sonar = Ultrasonic(C.TRIG_PIN, C.ECHO_PIN) if C.OBSTACLE_ON else None
        self.obstacle = Obstacle(C.STOP_CM, C.CLEAR_CM)
        self.follower = Follower(C.KP, C.KD, C.FAST, C.SLOW, C.CURVE_ERR, C.TURN_MAX, C.WHEEL_MIN,
                                 C.LOST_LEVEL, C.SIDE_ERR, C.BLIND_MS, C.LOST_ERR, C.RECENT_MS,
                                 C.SEARCH_SPEED, C.LOST_MS)

    # ---------- start-up calibration ----------

    def _sample(self, cal, ms):
        """Record both sensors for ms."""
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < ms:
            a, b = self.sensors.raw_u16()
            cal.add(a, b)
            time.sleep_ms(8)

    def _sweep_left(self, cal):
        """Spin left, recording both sensors, until the readings have changed and then held
        still: the sensors are then clear of the line. If nothing changes the robot is
        probably not turning, so push harder (twice at most). Returns (speed, ms at that speed)."""
        s = C.CAL_SPEED
        self.car.drive(-s, s)
        t0 = time.ticks_ms()
        t_change = -1                    # when a reading last moved outside its range
        t_speed = 0                      # when the speed was last raised
        while True:
            t = time.ticks_diff(time.ticks_ms(), t0)
            if t >= C.CAL_MAX_MS:
                break
            if t_change >= 0:
                if (t - t_speed >= C.CAL_LEFT_MS and
                        t - t_change >= max(C.CAL_SETTLE_MS, t_change - t_speed)):
                    break
            elif t - t_speed >= 400 and s < C.CAL_SPEED + 0.15:
                s += 0.1
                t_speed = t
                self.car.drive(-s, s)
            a, b = self.sensors.raw_u16()
            if cal.add(a, b):
                t_change = t
            time.sleep_ms(8)
        return s, t - t_speed

    def calibrate(self):
        """Sweep the sensors left then right across the line to learn white and black.
        The robot must start on the line. Returns False if it found no usable contrast."""
        self.led.on()
        gc.collect()
        cal = Calibration()
        s, ms = self._sweep_left(cal)            # left until clear of the line
        self.car.stop()
        self._sample(cal, 150)
        cal.end_left_sweep()
        self.car.drive(s, -s)                    # back across the line and as far again
        self._sample(cal, 2 * ms)
        self.car.stop()
        self._sample(cal, 150)

        gc.collect()
        levels = cal.levels(C.MIN_CONTRAST, C.IR_LINE_READS)
        if levels is None:
            both = self.sensors.load(C.CAL_FILE)
            if both is None:
                print("Calibration failed: the sweep saw no line. Put the robot on the line, press BOOTSEL.")
                return False
            print("The sweep saw no line: using the saved calibration.")
        else:
            both = cal.both_on_line(levels)
            if C.IR_AUTO_SIDES and cal.sides_ok(levels) is False:
                self.sensors.swap()
                levels = (levels[1], levels[0])
            self.sensors.levels = levels
            self.sensors.save(C.CAL_FILE, both)
        lv = self.sensors.levels
        print("Calibrated. left white {:.2f} black {:.2f} | right white {:.2f} black {:.2f}".format(
            lv[0][0], lv[0][1], lv[1][0], lv[1][1]))
        print("Sensor pins swapped: {}. Line covers both sensors: {}.".format(self.sensors.swapped, both))
        if hasattr(gc, "mem_free"):
            print("Free memory: {} bytes".format(gc.mem_free()))

        self.follower.wide = both
        self.follower.reset()
        self.follower.start_search(1)    # the sweep ended to the right of the line: turn back left
        return True

    # ---------- helpers ----------

    def _blink(self, now, period_ms):
        self.led.value((now // period_ms) & 1)

    def _wait_for_button(self, period_ms):
        """Blink until BOOTSEL is pressed."""
        while not bootsel_button():
            self._blink(time.ticks_ms(), period_ms)
            time.sleep_ms(50)
        self.led.on()
        time.sleep_ms(1000)              # time to let go of the robot

    # ---------- main loop ----------

    def run(self):
        """Wait, calibrate, then follow the line until stopped. Motors always end up off."""
        try:
            self.led.on()
            time.sleep_ms(C.START_DELAY_MS)
            while not self.calibrate():
                self._wait_for_button(FAST_BLINK_MS)
            self._loop()
        finally:
            self.car.stop()
            self.led.off()

    def _loop(self):
        gc.collect()                     # clear the calibration data before the timed loop
        f = self.follower
        paused = False
        blocked = False
        button_was = 0
        cm = None
        l = r = 0.0
        now = time.ticks_ms()
        next_t = last = t_ping = t_button = t_debug = t_gc = now
        while True:
            now = time.ticks_ms()
            dt = time.ticks_diff(now, last)
            last = now

            if time.ticks_diff(now, t_button) >= 50:
                t_button = now
                button = bootsel_button()
                if button and not button_was:
                    paused = not paused
                    f.hold()
                button_was = button

            if self.sonar and time.ticks_diff(now, t_ping) >= C.PING_EVERY_MS:
                t_ping = now
                cm = self.sonar.ping_cm(C.PING_TIMEOUT_US)
                blocked = self.obstacle.update(cm)

            nl, nr = self.sensors.read()
            if paused:
                l = r = 0.0
                self.car.stop()
                self._blink(now, SLOW_BLINK_MS)
            elif blocked:
                l = r = 0.0
                self.car.stop()
                f.hold()
                self._blink(now, FAST_BLINK_MS)
            else:
                l, r = f.step(nl, nr, dt)
                self.car.drive(l, r)
                if f.state == f.FOLLOW:
                    self.led.on()
                else:
                    self._blink(now, SLOW_BLINK_MS)

            if C.DEBUG and time.ticks_diff(now, t_debug) >= 200:
                t_debug = now
                print("{} L{:.2f} R{:.2f} e{:+.2f} -> l{:+.2f} r{:+.2f} cm {}{}".format(
                    f.state, nl, nr, nl - nr, l, r, cm,
                    " PAUSED" if paused else " BLOCKED" if blocked else ""))

            if time.ticks_diff(now, t_gc) >= 250:
                t_gc = now
                gc.collect()

            next_t = time.ticks_add(next_t, C.LOOP_MS)
            wait = time.ticks_diff(next_t, time.ticks_ms())
            if wait > 0:
                time.sleep_ms(wait)
            else:
                next_t = time.ticks_ms()   # running late: don't try to catch up
