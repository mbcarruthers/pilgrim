# Logic tests for src/follower.py. They run on the PC; no robot needed.
#   make test        (or: python3 tests/host/test_follower.py)

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "src"))

from follower import Calibration, Follower, Obstacle, darkness


def make(**kw):
    args = dict(kp=0.6, kd=0.0, fast=0.3, slow=0.1, curve_err=0.6, turn_max=0.6, wheel_min=0.0,
                lost_level=0.15, side_err=0.1, blind_ms=30, lost_err=0.3, recent_ms=400,
                search_speed=0.15, lost_ms=2000)
    args.update(kw)
    return Follower(**args)


def test_darkness_scales_and_clamps():
    assert darkness(0.15, 0.15, 0.80) == 0.0
    assert darkness(0.80, 0.15, 0.80) == 1.0
    assert abs(darkness(0.475, 0.15, 0.80) - 0.5) < 1e-9
    assert darkness(0.99, 0.15, 0.80) == 1.0
    assert darkness(0.20, 0.85, 0.20) == 1.0          # sensors that read low over black


def test_centred_drives_straight_at_full_speed():
    f = make()
    l, r = f.step(1.0, 1.0, 10)
    assert l == r == 0.3


def test_line_to_the_left_turns_left():
    f = make()
    l, r = f.step(1.0, 0.2, 10)                       # left sensor on the line, right nearly off
    assert l < r
    f = make()
    l, r = f.step(0.2, 1.0, 10)
    assert l > r


def test_slows_down_as_the_error_grows():
    f = make(kp=0.0)
    straight = f.step(1.0, 1.0, 10)[0]
    curve = f.step(1.0, 0.7, 10)[0]
    sharp = f.step(1.0, 0.0, 10)[0]
    assert straight > curve > sharp
    assert abs(sharp - 0.1) < 1e-9


def test_wheels_never_reverse_while_following():
    f = make()
    l, r = f.step(1.0, 0.0, 10)
    assert l == 0.0 and r > 0.0


def test_derivative_damps_a_fast_swing_and_skips_the_first_step():
    f = make(kd=0.02)
    f.step(1.0, 1.0, 10)
    l1, r1 = f.step(1.0, 0.8, 10)                     # error jumps 0 -> 0.2 in 10 ms
    g = make(kd=0.0)
    g.step(1.0, 1.0, 10)
    l0, r0 = g.step(1.0, 0.8, 10)
    assert (r1 - l1) > (r0 - l0)
    h = make(kd=0.02)
    assert h.step(1.0, 0.8, 10) == (l0, r0)           # no derivative on the very first step


def test_wide_line_lost_means_search_toward_last_side():
    f = make()
    f.step(1.0, 1.0, 10)
    f.step(1.0, 0.2, 10)                              # line drifting to the left
    for _ in range(5):
        l, r = f.step(0.0, 0.0, 10)
    assert f.state == f.LOST
    assert (l, r) == (-0.15, 0.15)                    # pivot left
    f = make()
    f.step(0.2, 1.0, 10)
    for _ in range(5):
        l, r = f.step(0.0, 0.0, 10)
    assert (l, r) == (0.15, -0.15)                    # pivot right


def test_small_error_still_sets_the_side_on_a_wide_line():
    f = make()
    f.step(0.83, 1.0, 10)                             # centred at a right-hand corner: e = -0.17
    for _ in range(5):
        l, r = f.step(0.0, 0.0, 10)
    assert f.state == f.LOST and l > 0 > r


def test_search_ends_when_the_line_is_found():
    f = make()
    f.step(1.0, 0.2, 10)
    for _ in range(10):
        f.step(0.0, 0.0, 10)
    assert f.state == f.LOST
    f.step(0.6, 0.0, 10)
    assert f.state == f.FOLLOW


def test_gives_up_then_restarts_when_put_back():
    f = make()
    f.step(1.0, 0.2, 10)
    for _ in range(300):
        out = f.step(0.0, 0.0, 10)
    assert f.state == f.GAVE_UP and out == (0.0, 0.0)
    assert f.step(0.3, 0.2, 10) == (0.0, 0.0)         # not clearly on a line yet
    l, r = f.step(1.0, 1.0, 10)
    assert f.state == f.FOLLOW and l > 0 and r > 0


def test_narrow_line_between_the_sensors_is_not_lost():
    f = make()
    f.wide = False
    for _ in range(300):                              # 3 s with neither sensor on the line
        l, r = f.step(0.02, 0.01, 10)
    assert f.state == f.FOLLOW and l > 0.25 and r > 0.25


def test_narrow_line_drove_off_is_lost():
    f = make()
    f.wide = False
    f.step(0.0, 0.0, 10)
    f.step(0.0, 0.9, 10)                              # line went under the right sensor...
    l, r = f.step(0.0, 0.0, 10)                       # ...and out the other side
    assert f.state == f.LOST and l > 0 > r


def test_start_search_pivots_the_asked_way():
    f = make()
    f.start_search(1)
    assert f.step(0.0, 0.0, 10) == (-0.15, 0.15)


def test_obstacle_needs_two_near_readings_and_clears_beyond_clear_distance():
    o = Obstacle(15, 25)
    assert o.update(10) is False
    assert o.update(10) is True
    assert o.update(20) is True                       # between stop and clear: stay stopped
    assert o.update(30) is False
    assert o.update(10) is False                      # needs two again


def test_obstacle_no_echo_never_blocks():
    o = Obstacle(15, 25)
    for _ in range(10):
        assert o.update(None) is False
    o.update(10)
    o.update(10)
    assert o.update(None) is False                    # obstacle gone: nothing in range


def sweep(line_high=True, swapped=False, wide=True):
    """Readings from a left sweep off a line, then a right sweep back across it and beyond."""
    white, black = (0.15, 0.80) if line_high else (0.85, 0.20)
    if wide:    # both start on the line; the left sensor leaves it first
        left = [black] * 5 + [white] * 40
        right = [black] * 20 + [white] * 25
        left += [white] * 20 + [black] * 20 + [white] * 50
        right += [white] * 5 + [black] * 20 + [white] * 65
    else:       # line between the sensors; only the right one crosses it going left
        left = [white] * 45
        right = [white] * 10 + [black] * 8 + [white] * 27
        left += [white] * 40 + [black] * 8 + [white] * 42
        right += [white] * 27 + [black] * 8 + [white] * 55
    cal = Calibration()
    for i in range(len(left)):
        a, b = (right[i], left[i]) if swapped else (left[i], right[i])
        add(cal, a, b)
        if i == 44:
            cal.end_left_sweep()
    return cal


def add(cal, a, b):
    """Record two readings given as fractions of full scale."""
    return cal.add(int(a * 65535), int(b * 65535))


def close(levels, expected):
    return all(abs(levels[i][j] - expected[i][j]) < 0.01 for i in (0, 1) for j in (0, 1))


def test_calibration_finds_levels_for_either_polarity():
    assert close(sweep(True).levels(0.1), ((0.15, 0.80), (0.15, 0.80)))
    assert close(sweep(False).levels(0.1), ((0.85, 0.20), (0.85, 0.20)))


def test_calibration_polarity_can_be_forced():
    assert close(sweep(True).levels(0.1, "low"), ((0.80, 0.15), (0.80, 0.15)))


def test_calibration_fails_without_contrast():
    cal = Calibration()
    for i in range(100):
        add(cal, 0.15 + (i % 3) * 0.01, 0.16)
    assert cal.levels(0.1) is None


def test_calibration_reports_a_change_only_outside_the_range_so_far():
    cal = Calibration()
    assert add(cal, 0.80, 0.80) is False
    assert add(cal, 0.81, 0.79) is False              # noise
    assert add(cal, 0.50, 0.80) is True               # left sensor leaving the line
    assert add(cal, 0.52, 0.80) is False
    assert add(cal, 0.15, 0.80) is True


def test_sides_detected_from_the_left_sweep():
    for wide in (True, False):
        cal = sweep(wide=wide)
        assert cal.sides_ok(cal.levels(0.1)) is True
        cal = sweep(swapped=True, wide=wide)
        assert cal.sides_ok(cal.levels(0.1)) is False


def test_sides_detected_from_the_right_sweep_alone():
    for swapped in (False, True):                     # sensors start off the line, line to the right
        cal = Calibration()
        for i in range(45):
            add(cal, 0.15, 0.15)
        cal.end_left_sweep()
        for i in range(90):                           # sweeping right, the right sensor gets there first
            left = 0.80 if 30 < i < 50 else 0.15
            right = 0.80 if 20 < i < 40 else 0.15
            add(cal, right, left) if swapped else add(cal, left, right)
        assert cal.sides_ok(cal.levels(0.1)) is (not swapped)


def test_sides_unclear_when_both_sensors_cross_together():
    cal = Calibration()
    for i in range(45):
        add(cal, 0.15, 0.15)
    cal.end_left_sweep()
    for i in range(90):
        v = 0.80 if 20 < i < 40 else 0.15
        add(cal, v, v)
    assert cal.sides_ok(cal.levels(0.1)) is None


def test_recording_stops_quietly_when_the_buffers_are_full():
    cal = Calibration(size=30)
    for i in range(100):
        add(cal, 0.15 if i < 20 else 0.80, 0.15 if i < 25 else 0.80)
    assert cal.n == 30
    assert close(cal.levels(0.1), ((0.15, 0.80), (0.15, 0.80)))


def test_line_width_detected():
    cal = sweep(wide=True)
    assert cal.both_on_line(cal.levels(0.1)) is True
    cal = sweep(wide=False)
    assert cal.both_on_line(cal.levels(0.1)) is False


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print("ok  ", name)
    print("{} logic tests passed".format(len(tests)))
