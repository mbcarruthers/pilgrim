# Line-following logic. No hardware in here, so it also runs (and is tested) on the PC.
#
# Each line sensor gives a darkness n: 0 over white, 1 over the black line.
# Error e = n_left - n_right, from -1 to 1. Positive means the line is to the left.
# This works whether the two sensors straddle the line or both sit on it.

from array import array


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def darkness(raw, white, black):
    """0 over white, 1 over black, for one sensor's raw reading."""
    return clamp((raw - white) / (black - white), 0.0, 1.0)


def _zeros(size):
    return array("H", (0 for _ in range(size)))


class Calibration:
    """Readings from both sensors while the robot sweeps them left, then right.
    Readings are raw ADC counts (0-65535) kept in two fixed buffers, so recording
    them uses no extra memory on the Pico."""

    STEP = 2600                          # counts (4% of full scale) outside the range so far = "a change"

    def __init__(self, size=800):
        self.a = _zeros(size)            # sensor wired as "left" in config
        self.b = _zeros(size)            # sensor wired as "right" in config
        self.n = 0
        self.left_end = 0
        self._range = [0, 0, 0, 0]       # a_lo, a_hi, b_lo, b_hi

    def add(self, raw_a, raw_b):
        """Record one reading of each sensor. True if either moved outside its range so far."""
        n = self.n
        if n < len(self.a):
            self.a[n] = raw_a
            self.b[n] = raw_b
            self.n = n + 1
        r = self._range
        if n == 0:
            r[0] = r[1] = raw_a
            r[2] = r[3] = raw_b
            return False
        changed = False
        if raw_a < r[0] - self.STEP:
            r[0] = raw_a
            changed = True
        elif raw_a > r[1] + self.STEP:
            r[1] = raw_a
            changed = True
        if raw_b < r[2] - self.STEP:
            r[2] = raw_b
            changed = True
        elif raw_b > r[3] + self.STEP:
            r[3] = raw_b
            changed = True
        return changed

    def end_left_sweep(self):
        self.left_end = self.n

    def _low_high(self, buf):
        """Lowest and highest reading, ignoring one stray spike at either end."""
        lo = lo2 = 65535
        hi = hi2 = 0
        for i in range(self.n):
            x = buf[i]
            if x < lo:
                lo2 = lo
                lo = x
            elif x < lo2:
                lo2 = x
            if x > hi:
                hi2 = hi
                hi = x
            elif x > hi2:
                hi2 = x
        return (lo2, hi2) if self.n >= 50 else (lo, hi)

    @staticmethod
    def _median(buf, start, end, vmin, vmax):
        """Median of the readings buf[start:end] that lie in vmin..vmax (to about 1%)."""
        bins = [0] * 64
        width = (vmax - vmin) // 64 + 1
        count = 0
        for i in range(start, end):
            x = buf[i]
            if vmin <= x <= vmax:
                bins[(x - vmin) // width] += 1
                count += 1
        if count == 0:
            return (vmin + vmax) // 2
        seen = 0
        for k in range(64):
            seen += bins[k]
            if seen * 2 > count:
                return min(vmax, vmin + k * width + width // 2)
        return vmax

    def levels(self, min_contrast, line_reads="auto"):
        """((white, black), (white, black)) for the two sensors as fractions of full scale,
        or None if either sensor saw too little contrast.
        line_reads: "high" or "low" if A0 over black is known, else "auto"."""
        n = self.n
        if n < 20:
            return None
        la, ha = self._low_high(self.a)
        lb, hb = self._low_high(self.b)
        need = min_contrast * 65535
        if ha - la < need or hb - lb < need:
            return None
        if line_reads == "auto":
            # Three clues that the line reads high, decided together for both sensors:
            # these modules normally do; the left sweep ends over background;
            # and most of the whole sweep is background.
            votes = 1
            t0 = max(0, self.left_end - 10) if self.left_end else 0
            t1 = self.left_end if self.left_end else n
            for buf, lo, hi in ((self.a, la, ha), (self.b, lb, hb)):
                tail = self._median(buf, t0, t1, 0, 65535)
                mid = self._median(buf, 0, n, 0, 65535)
                votes += 1 if hi - tail >= tail - lo else 0
                votes += 1 if hi - mid >= mid - lo else 0
            line_reads = "high" if votes >= 3 else "low"
        out = []
        for buf, lo, hi in ((self.a, la, ha), (self.b, lb, hb)):
            half = (lo + hi) // 2
            if line_reads == "high":
                white = self._median(buf, 0, n, lo, half)
                out.append((white / 65535, hi / 65535))
            else:
                white = self._median(buf, 0, n, half, hi)
                out.append((white / 65535, lo / 65535))
        return out[0], out[1]

    @staticmethod
    def _crossing(buf, level, start, end):
        """(entered, left): when this sensor came onto the line and when it went off it
        during readings start..end. None for one that did not happen inside that window."""
        white, black = level
        mid = (white + black) * 32767.5
        high = black > white
        first = last = None
        for i in range(start, end):
            if (buf[i] > mid) == high:
                if first is None:
                    first = i
                last = i
        if first is None:
            return None, None
        return (None if first <= start + 1 else first), (None if last >= end - 3 else last)

    def sides_ok(self, levels):
        """True if the sensors are on the sides config says, False if swapped, None if unclear.
        Sweeping left, the right-hand sensor reaches and leaves the line last;
        sweeping right, it reaches and leaves it first."""
        votes = 0
        for start, end, leftward in ((0, self.left_end, True), (self.left_end, self.n, False)):
            ca = self._crossing(self.a, levels[0], start, end)
            cb = self._crossing(self.b, levels[1], start, end)
            for k in (0, 1):
                if ca[k] is None or cb[k] is None or abs(ca[k] - cb[k]) < 2:
                    continue
                votes += 1 if (cb[k] > ca[k]) == leftward else -1
        if votes == 0:
            return None
        return votes > 0

    def both_on_line(self, levels):
        """True if both sensors were ever over the line together (line wider than their spacing)."""
        mid_a = (levels[0][0] + levels[0][1]) * 32767.5
        mid_b = (levels[1][0] + levels[1][1]) * 32767.5
        high_a = levels[0][1] > levels[0][0]
        high_b = levels[1][1] > levels[1][0]
        for i in range(self.n):
            if (self.a[i] > mid_a) == high_a and (self.b[i] > mid_b) == high_b:
                return True
        return False


class Follower:
    """One control step at a time: sensor darkness in, wheel commands (-1 to 1) out."""

    FOLLOW = "follow"
    LOST = "lost"
    GAVE_UP = "gave up"

    def __init__(self, kp, kd, fast, slow, curve_err, turn_max, wheel_min,
                 lost_level, side_err, blind_ms, lost_err, recent_ms, search_speed, lost_ms):
        self.kp, self.kd = kp, kd
        self.fast, self.slow, self.curve_err = fast, slow, curve_err
        self.turn_max, self.wheel_min = turn_max, wheel_min
        self.lost_level = lost_level
        self.side_err, self.blind_ms = side_err, blind_ms        # wide line
        self.lost_err, self.recent_ms = lost_err, recent_ms      # narrow line
        self.search_speed, self.lost_ms = search_speed, lost_ms
        self.wide = True                 # the line covers both sensors (set from calibration)
        self.reset()

    def reset(self):
        self.state = self.FOLLOW
        self.e = 0.0
        self.last_e = 0.0
        self.fresh = True                # next step has no valid derivative
        self.side = 0                    # +1 line last seen to the left, -1 to the right
        self.since_side = 100000         # ms since the line was clearly at one side
        self.blind_for = 0               # ms with no line in view while following
        self.lost_for = 0

    def hold(self):
        """Call while the robot is held still (paused, obstacle) so steering restarts cleanly."""
        self.fresh = True

    def start_search(self, side):
        """Begin by pivoting toward `side` (+1 left, -1 right) to find the line."""
        self.state = self.LOST
        self.side = side
        self.lost_for = 0
        self.fresh = True

    def step(self, n_left, n_right, dt_ms):
        e = n_left - n_right
        self.e = e
        no_line = n_left < self.lost_level and n_right < self.lost_level

        if self.state == self.GAVE_UP:   # stopped; start again when put back on a line
            if max(n_left, n_right) < 0.5:
                return 0.0, 0.0
            self.state = self.FOLLOW
            self.fresh = True

        if no_line and self.state == self.FOLLOW:
            self.blind_for += dt_ms
            if self.wide:                # a wide line is never out of view while on it
                lost = self.blind_for >= self.blind_ms
            else:                        # a narrow line vanishes between the sensors when centred
                lost = self.since_side <= self.recent_ms
            if lost:
                self.state = self.LOST
                self.lost_for = 0

        if self.state == self.LOST:
            if no_line:
                self.lost_for += dt_ms
                if self.lost_for >= self.lost_ms:
                    self.state = self.GAVE_UP
                    return 0.0, 0.0
                s = self.search_speed
                return (-s, s) if self.side >= 0 else (s, -s)
            self.state = self.FOLLOW     # found it
            self.fresh = True

        if not no_line:
            self.blind_for = 0
        if abs(e) >= (self.side_err if self.wide else self.lost_err):
            self.side = 1 if e > 0 else -1
            self.since_side = 0
        elif self.since_side < 100000:
            self.since_side += dt_ms

        de = 0.0 if (self.fresh or dt_ms <= 0) else (e - self.last_e) * 1000.0 / dt_ms
        self.fresh = False
        self.last_e = e
        turn = clamp(self.kp * e + self.kd * de, -self.turn_max, self.turn_max)
        base = self.fast - (self.fast - self.slow) * min(1.0, abs(e) / self.curve_err)
        return (clamp(base - turn, self.wheel_min, 1.0),
                clamp(base + turn, self.wheel_min, 1.0))


class Obstacle:
    """Blocked after `confirm` readings nearer than stop_cm; clear again beyond clear_cm.
    No echo (None) means nothing in range, so an unpowered sensor never blocks."""

    def __init__(self, stop_cm, clear_cm, confirm=2):
        self.stop_cm, self.clear_cm, self.confirm = stop_cm, clear_cm, confirm
        self.blocked = False
        self.near = 0

    def update(self, cm):
        if cm is None or cm > self.clear_cm:
            self.blocked = False
            self.near = 0
        elif cm < self.stop_cm:
            self.near += 1
            if self.near >= self.confirm:
                self.blocked = True
        return self.blocked
