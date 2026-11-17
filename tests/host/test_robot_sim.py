# Whole-robot tests: the real src/ code drives a simulated robot around a tape track.
# They run on the PC; no robot needed.
#   make test        (or: python3 tests/host/test_robot_sim.py)

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sim

LAP = 2 * 0.8 + 2 * math.pi * 0.25          # one lap of the default oval, in metres


def oval(**kw):
    return sim.World(sim.Stadium(), **kw)


def worst_mm(world, after_ms, before_ms=1e12):
    """Largest distance of the sensor midpoint from the line's centre in that time window."""
    return max(e for (t, e, m) in world.trace if after_ms < t < before_ms) * 1000


def moving(world, after_ms, before_ms):
    return any(m for (t, e, m) in world.trace if after_ms < t < before_ms)


def teleport(x, y, heading):
    def go(w):
        w.x, w.y, w.th = x, y, heading
        w.vl = w.vr = 0.0
    return go


def follows(world, robot, laps=1.5, limit_mm=15):
    assert robot.follower.state == robot.follower.FOLLOW, robot.follower.state
    assert world.travel > laps * LAP, "only {:.1f} laps".format(world.travel / LAP)
    assert worst_mm(world, 6000) < limit_mm, "{:.0f} mm off the line".format(worst_mm(world, 6000))


def test_follows_the_oval_anticlockwise():
    w = oval()
    follows(w, sim.run(w))


def test_follows_the_oval_clockwise():
    w = oval(x=0.6, heading=math.pi)
    follows(w, sim.run(w))


def test_sensors_wired_the_other_way_round_are_detected():
    w = oval(sensors_swapped=True)
    r = sim.run(w)
    assert r.sensors.swapped is True
    follows(w, r)


def test_sensors_that_read_low_over_black():
    w = oval(black_reads_high=False)
    r = sim.run(w)
    assert r.sensors.levels[0][1] < r.sensors.levels[0][0]
    follows(w, r)


def test_narrow_line_between_the_sensors():
    w = oval(tape_mm=10)
    r = sim.run(w)
    assert r.follower.wide is False
    follows(w, r)


def test_wide_line():
    w = oval(tape_mm=25)
    r = sim.run(w)
    assert r.follower.wide is True
    follows(w, r)


def test_tight_curves():
    w = sim.World(sim.Stadium(R=0.12))
    r = sim.run(w)
    assert r.follower.state == r.follower.FOLLOW
    assert w.travel > 5.0 and worst_mm(w, 6000) < 15


def test_worn_batteries():
    w = oval(true_deadband=(0.62, 0.46), vmax=(0.5, 0.5), end_ms=40000)
    follows(w, sim.run(w), laps=1.0)


def test_wheels_stickier_than_measured_still_calibrate():
    w = oval(true_deadband=(0.70, 0.55), end_ms=60000)    # the start-up spin does not move at first
    r = sim.run(w)
    assert r.sensors.levels is not None
    follows(w, r, laps=0.4)


def test_fresh_batteries():
    w = oval(true_deadband=(0.25, 0.20), vmax=(0.70, 0.75))
    follows(w, sim.run(w), laps=2.5)


def test_square_with_sharp_corners_both_ways():
    square = sim.Polyline([(0, 0), (0.7, 0), (0.7, 0.7), (0, 0.7)])
    for x, heading in ((0.2, 0.0), (0.5, math.pi)):
        w = sim.World(square, x=x, heading=heading, end_ms=40000)
        r = sim.run(w)
        assert r.follower.state != r.follower.GAVE_UP
        assert w.travel > 2.5 * 2.8, "only {:.1f} laps".format(w.travel / 2.8)
        assert worst_mm(w, 6000) < 60                 # it cuts or overshoots corners, then recovers


def test_goes_straight_through_a_crossing():
    crossing = sim.Polyline([(0.5, -0.3), (0.5, 0.3)])
    w = sim.World(sim.Both(sim.Stadium(), crossing))
    follows(w, sim.run(w))


def test_stops_for_an_obstacle_and_resumes():
    w = oval()
    w.obstacle = lambda t: 10 if 10000 < t < 14000 else None
    r = sim.run(w)
    assert moving(w, 8000, 10000)
    assert not moving(w, 10800, 14000), "kept moving with an obstacle 10 cm ahead"
    assert moving(w, 14500, 16000), "did not resume after the obstacle was removed"
    follows(w, r, laps=1.0)


def test_far_object_does_not_stop_it():
    w = oval()
    w.obstacle = lambda t: 40
    follows(w, sim.run(w))


def test_obstacle_sensor_switched_off_in_config():
    w = oval()
    w.obstacle = lambda t: 5
    r = sim.run(w, OBSTACLE_ON=False)
    assert r.sonar is None
    follows(w, r)


def test_lifted_off_the_line_gives_up_then_resumes_when_put_back():
    w = oval(end_ms=35000)
    w.events = [(10000, teleport(5, 5, 0)), (18000, teleport(0.2, 0.0, 0.0))]
    r = sim.run(w)
    assert not moving(w, 13500, 18000), "still driving 3.5 s after losing the line"
    assert moving(w, 19000, 21000)
    assert r.follower.state == r.follower.FOLLOW
    assert worst_mm(w, 22000) < 15


def test_calibration_fails_safely_and_retries_on_bootsel():
    w = oval(x=5, y=5, end_ms=35000)                  # starts nowhere near a line
    w.events = [(9000, teleport(0.2, 0.0, 0.0))]
    w.button = lambda t: 1 if 10000 < t < 10300 else 0
    r = sim.run(w)
    assert not moving(w, 9000, 10000), "drove off after a failed calibration"
    assert r.follower.state == r.follower.FOLLOW
    assert worst_mm(w, 20000) < 15


def test_falls_back_to_the_saved_calibration():
    w = oval(end_ms=8000)
    sim.run(w)                                        # a good run saves its calibration
    w = oval(x=5, y=5, end_ms=30000)                  # then a start away from the line
    w.events = [(12000, teleport(0.2, 0.0, 0.0))]
    r = sim.run(w, KEEP_CAL=True)
    assert r.follower.state == r.follower.FOLLOW
    assert worst_mm(w, 16000) < 15


def test_bootsel_pauses_and_resumes():
    w = oval()
    w.button = lambda t: 1 if (10000 < t < 10200 or 14000 < t < 14200) else 0
    r = sim.run(w)
    assert not moving(w, 10800, 14000)
    assert moving(w, 14500, 16000)
    follows(w, r, laps=1.0)


def test_motors_end_up_off():
    w = oval(end_ms=9000)
    sim.run(w)                                        # the run is cut off like Ctrl-C
    assert w.motors_off() and w.led == 0


def test_control_loop_keeps_its_period():
    import follower
    w = oval(end_ms=15000)
    w.obstacle = lambda t: None                       # worst case: every ping times out
    times = []
    step = follower.Follower.step

    def spy(self, a, b, dt):
        times.append(dt)
        return step(self, a, b, dt)
    follower.Follower.step = spy
    try:
        sim.run(w)
    finally:
        follower.Follower.step = step
    assert max(times) <= 22, max(times)               # one late loop per ping at most
    assert sum(times) / len(times) < 11.5


if __name__ == "__main__":
    import contextlib
    import io
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        with contextlib.redirect_stdout(io.StringIO()):
            fn()
        print("ok  ", name)
    leftover = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_cal_test.json")
    if os.path.exists(leftover):
        os.remove(leftover)
    print("{} robot simulation tests passed".format(len(tests)))
