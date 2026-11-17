# Bring-up and measurement tool for the lab4 line follower (MicroPython, RP2040)
#
# Keep this file in bringup/ next to src/, so `make deploy` never copies it.
# From the project folder:
#     mpremote mount bringup repl
#     >>> import bringup as b
#     >>> b.help()
#
# Results are saved after every test to bringup/results.json on the computer.
# b.summary() prints them as a Markdown table and writes bringup/results.md.
# Ctrl-C stops any test. b.stop() turns the motors off.

import sys
import time
import json
import select
from machine import Pin, PWM, ADC, time_pulse_us

# Wiring (from the pin map)
#   GP0 -> L9110S A-1A   GP1 -> L9110S A-1B   (motor channel A)
#   GP2 -> L9110S B-1A   GP3 -> L9110S B-1B   (motor channel B)
#   GP26 <- IR tracking sensor (left): A0   GP28 <- IR tracking sensor (right): A0
#            (set IR_MODE = "digital" below if D0 is wired there instead)
#   GP19 -> Ultrasonic ranging sensor: Trig   GP18 <- Echo (through a 1:2 divider)
CHANNEL_A = (0, 1)
CHANNEL_B = (2, 3)
IR_PINS = (("left", 26), ("right", 28))   # IR tracking sensors (4-pin TCRT5000 modules)
IR_MODE = "analog"       # "analog": A0 wired to those pins. "digital": D0 wired there instead.
TRIG_PIN = 19
ECHO_PIN = 18
PWM_FREQ = 1000          # same as motor.py
CM_PER_US = 0.0343       # speed of sound

RESULTS_FILE = "results.json"
SUMMARY_FILE = "results.md"
R = {}

_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)
_pwm = {}


# ---------- helpers ----------

def _load():
    global R
    try:
        with open(RESULTS_FILE) as f:
            R = json.load(f)
        print("Loaded earlier results from", RESULTS_FILE)
    except (OSError, ValueError):
        R = {}


def _save():
    try:
        with open(RESULTS_FILE, "w") as f:
            json.dump(R, f)
    except OSError:
        print("(Could not save to the computer; results are kept in memory until reset.)")


def _key():
    """True if a key was pressed since the last check (the key is consumed)."""
    if _poll.poll(0):
        sys.stdin.read(1)
        return True
    return False


def _flush():
    while _poll.poll(0):
        sys.stdin.read(1)


def _ask_float(prompt):
    while True:
        s = input(prompt + " ").strip()
        try:
            return float(s)
        except ValueError:
            print("  Type a number.")


def _ask_choice(prompt, choices):
    while True:
        s = input("{} [{}] ".format(prompt, "/".join(choices))).strip().lower()
        if s in choices:
            return s
        print("  Type one of: " + ", ".join(choices))


def _pwm_pin(pin):
    if pin not in _pwm:
        p = PWM(Pin(pin))
        p.freq(PWM_FREQ)
        p.duty_u16(0)
        _pwm[pin] = p
    return _pwm[pin]


def _duty(pin, frac):
    frac = max(0.0, min(1.0, frac))
    _pwm_pin(pin).duty_u16(int(frac * 65535))


def stop():
    """Turn both motors off."""
    for pin in CHANNEL_A + CHANNEL_B:
        _duty(pin, 0)


def _need_map():
    m = R.get("motor_map")
    if not m:
        print("Run b.pins() first, and fix any wiring problem it reports.")
    return m


def _trims():
    t = R.get("trim") or {}
    tl, tr = t.get("left", 1.0), t.get("right", 1.0)
    if not (0 < tl <= 1 and 0 < tr <= 1):   # an impossible value from an earlier bad run
        return 1.0, 1.0
    return tl, tr


# ---------- 1. board ----------

def info():
    """Firmware version, BOOTSEL support, onboard LED blink."""
    import os
    u = os.uname()
    print("Firmware:", u.version)
    print("Board:   ", u.machine)
    try:
        import rp2
        rp2.bootsel_button
        bootsel = "yes"
    except (ImportError, AttributeError):
        bootsel = "no"
    print("rp2.bootsel_button() available:", bootsel)
    print("The onboard LED should blink 3 times.")
    led = Pin("LED", Pin.OUT)
    for _ in range(6):
        led.toggle()
        time.sleep_ms(200)
    led.off()
    blinked = _ask_choice("Did the onboard LED blink?", ("y", "n"))
    R["firmware"] = u.version
    R["board"] = u.machine
    R["bootsel_button"] = bootsel
    R["onboard_led"] = "works" if blinked == "y" else "did not blink"
    _save()


# ---------- 2. motor pins ----------

def pins(duty=0.6, secs=1.5):
    """Drive each motor input alone and record which wheel turns which way.
    The robot must be lifted so both wheels spin freely."""
    print("Lift the robot so both wheels spin freely. Battery pack on.")
    print("'forward' = the wheel turns the way that would roll the robot toward its IR tracking sensor end.")
    input("Press Enter to start...")
    obs = {}
    try:
        for pin in CHANNEL_A + CHANNEL_B:
            stop()
            print("\nGP{} on for {} s...".format(pin, secs))
            _duty(pin, duty)
            time.sleep(secs)
            stop()
            wheel = _ask_choice("  Which wheel turned?", ("left", "right", "none", "both"))
            way = "-"
            if wheel in ("left", "right"):
                way = _ask_choice("  Which way?", ("forward", "backward"))
            obs[pin] = (wheel, way)
    finally:
        stop()
    R["pins"] = {"GP{}".format(p): "{} {}".format(w, d) for p, (w, d) in obs.items()}
    R["motor_map"] = _map_from(obs)
    _save()


def _map_from(obs):
    problems = []
    wheels = {"left": {}, "right": {}}
    for pin, (wheel, way) in obs.items():
        if wheel == "none":
            problems.append("GP{} turned nothing: check its wire to the L9110S, and the battery pack.".format(pin))
        elif wheel == "both":
            problems.append("GP{} turned both wheels: two inputs are probably touching.".format(pin))
        elif way in wheels[wheel]:
            problems.append("GP{} and GP{} both turn the {} wheel {}.".format(wheels[wheel][way], pin, wheel, way))
        else:
            wheels[wheel][way] = pin
    m = {}
    for wheel in ("left", "right"):
        w = wheels[wheel]
        if "forward" not in w or "backward" not in w:
            problems.append("The {} wheel did not get one forward and one backward pin.".format(wheel))
            continue
        pair = tuple(sorted((w["forward"], w["backward"])))
        if pair not in (CHANNEL_A, CHANNEL_B):
            problems.append("The {} wheel is driven by GP{} and GP{}, which are on different L9110S channels.".format(wheel, *pair))
            continue
        pin_a, pin_b = pair
        m[wheel] = {"pin_a": pin_a, "pin_b": pin_b, "forward_pin": w["forward"],
                    "invert": w["forward"] != pin_a}
    if problems:
        print("\nWIRING PROBLEMS:")
        for p in problems:
            print("  - " + p)
        print("Fix them, then run b.pins() again.")
        return None
    print("\nMotor map (use in motor.py / car.py):")
    for wheel in ("right", "left"):
        x = m[wheel]
        print("  {:5s} wheel: Motor({}, {}, invert={})   forward = PWM on GP{}".format(
            wheel, x["pin_a"], x["pin_b"], x["invert"], x["forward_pin"]))
    return m


# ---------- 3. deadband ----------

def deadband(trials=2, step=0.01, every_ms=150):
    """Ramp each wheel's power up slowly on the floor. Press Enter the moment it starts turning."""
    m = _need_map()
    if not m:
        return
    print("Put the robot on the floor or the track.")
    print("One wheel at a time ramps up slowly. Press Enter the moment that wheel starts turning.")
    input("Press Enter to start...")
    res = {}
    try:
        for wheel in ("left", "right"):
            pin = m[wheel]["forward_pin"]
            found = []
            for t in range(trials):
                _flush()
                print("\n{} wheel, try {} of {}".format(wheel, t + 1, trials))
                d = 0.0
                hit = None
                while d <= 1.0 and hit is None:
                    _duty(pin, d)
                    print("\r  duty {:3d}%".format(int(d * 100)), end="")
                    t0 = time.ticks_ms()
                    while time.ticks_diff(time.ticks_ms(), t0) < every_ms:
                        if _key():
                            hit = d
                            break
                        time.sleep_ms(5)
                    d += step
                stop()
                print()
                if hit is None:
                    print("  Reached 100% with no key press.")
                else:
                    print("  started at {:.0f}%".format(hit * 100))
                    found.append(hit)
                time.sleep(1)
            # reaction time only ever makes the reading too high, so keep the lowest
            res[wheel] = round(min(found), 2) if found else None
    finally:
        stop()
    print("\nDeadband (fraction of full duty): left {}, right {}".format(res["left"], res["right"]))
    R["deadband"] = res
    _save()


# ---------- 4. straight line, trim and speed ----------

def straight(duty=0.8, secs=2.0):
    """Drive straight at one duty, measure distance and drift, and work out the trim.
    The duty must be above both wheels' deadband, or one wheel won't turn."""
    m = _need_map()
    if not m:
        return
    db = [v for v in (R.get("deadband") or {}).values() if v]
    if db and duty < max(db) + 0.05:
        print("{:.0f}% is too close to the deadband ({:.0f}%): the weaker wheel won't turn.".format(
            duty * 100, max(db) * 100))
        print("Use b.straight({:.2f}) or higher.".format(min(1.0, max(db) + 0.1)))
        return
    if "wheel_track_mm" not in R:
        R["wheel_track_mm"] = _ask_float("Distance between the centers of the two drive wheels (mm)?")
    W = R["wheel_track_mm"] / 10.0
    lf, rf = m["left"]["forward_pin"], m["right"]["forward_pin"]
    while True:
        tl, tr = _trims()
        print("\nMark the start. Point the robot along a straight edge (tape, floor seam).")
        input("Press Enter to drive {:.1f} s at {:.0f}% (trim left {:.3f}, right {:.3f})...".format(
            secs, duty * 100, tl, tr))
        try:
            _duty(lf, duty * tl)
            _duty(rf, duty * tr)
            time.sleep(secs)
        finally:
            stop()
        D = _ask_float("Distance driven forward (cm)?")
        y = _ask_float("Sideways drift at the end (cm; + = veered right, - = veered left)?")
        if D < 20:
            print("  Under 20 cm is too short to measure drift: check both wheels turned,")
            print("  then use more power or more seconds, e.g. b.straight(0.9, 3).")
            continue
        speeds = R.get("speed_cm_s") or {}
        speeds["{:.2f}".format(duty)] = round(D / secs, 1)
        R["speed_cm_s"] = speeds
        # veering right means the left wheel is faster: (vL - vR) / v = 2 y W / D^2
        f = 2 * y * W / (D * D)
        f = max(-0.5, min(0.5, f))   # one run never changes a wheel by more than half
        if f > 0:
            tl *= (1 - f)
        else:
            tr *= (1 + f)
        top = max(tl, tr)
        tl, tr = tl / top, tr / top
        R["trim"] = {"left": round(tl, 3), "right": round(tr, 3)}
        R["straight_drift_cm"] = "{} over {} cm".format(y, D)
        _save()
        print("  Speed {:.1f} cm/s. New trim: left {:.3f}, right {:.3f}".format(D / secs, tl, tr))
        if abs(y) <= 2 and D >= 80:
            print("  Within 2 cm over 80+ cm: good enough.")
        if _ask_choice("Run again with the new trim?", ("y", "n")) == "n":
            break


# ---------- 5. IR tracking sensors ----------

def _ir():
    """[(name, read)]. read() gives 0-100 (% of 3.3 V) in analog mode, or 0/1 in digital mode."""
    out = []
    for name, n in IR_PINS:
        if IR_MODE == "analog":
            adc = ADC(Pin(n))
            out.append((name, lambda a=adc: a.read_u16() * 100 / 65535))
        else:
            out.append((name, Pin(n, Pin.IN, Pin.PULL_UP).value))
    return out


def _on_line(name, v):
    """True if an analog reading v is on the line, using the threshold b.ir() recorded."""
    x = (R.get("ir") or {}).get(name) or {}
    if "threshold" not in x:
        return None
    return (v > x["threshold"]) == x["line_reads_higher"]


def ir_live():
    """Show the IR tracking sensor outputs until you press Enter."""
    sens = _ir()
    _flush()
    if IR_MODE == "analog":
        print("Analog, % of 3.3 V. After b.ir() it also shows LINE or white. Press Enter to stop.")
    else:
        print("Digital, 0 or 1. Press Enter to stop.")
    while not _key():
        parts = []
        for n, read in sens:
            v = read()
            if IR_MODE == "analog":
                s = "{} {:5.1f}%".format(n, v)
                on = _on_line(n, v)
                if on is not None:
                    s += " LINE " if on else " white"
            else:
                s = "{} {}".format(n, v)
            parts.append(s)
        print("\r  " + "   ".join(parts) + "   ", end="")
        time.sleep_ms(50)
    print()


def _sample_digital(read, n=40):
    """n readings over about 0.4 s: majority value and how often it changed."""
    ones = 0
    flips = 0
    last = read()
    for _ in range(n):
        v = read()
        ones += v
        if v != last:
            flips += 1
        last = v
        time.sleep_ms(10)
    return (1 if ones * 2 > n else 0), flips


def _sample_analog(read, n=40):
    """n readings over about 0.4 s: mean, lowest, highest (all % of 3.3 V)."""
    total = 0.0
    lo = 100.0
    hi = 0.0
    for _ in range(n):
        v = read()
        total += v
        lo = min(lo, v)
        hi = max(hi, v)
        time.sleep_ms(10)
    return total / n, lo, hi


def ir():
    """Record each IR tracking sensor over white and over the black line, at the mounted height."""
    sens = _ir()
    print("IR tracking sensors mounted at their final height, wired to 3V3, GND, GP26 and GP28.")
    if IR_MODE == "digital":
        print("Set each module's potentiometer first with b.ir_live(): its LED must change")
        print("as it crosses the line edge.")
    res = {}
    for name, read in sens:
        input("\nPut the {} IR tracking sensor over WHITE, then press Enter...".format(name.upper()))
        if IR_MODE == "analog":
            w, wlo, whi = _sample_analog(read)
        else:
            w, fw = _sample_digital(read)
        input("Put the {} IR tracking sensor over the BLACK line, then press Enter...".format(name.upper()))
        if IR_MODE == "analog":
            k, klo, khi = _sample_analog(read)
            higher = k > w
            gap = (klo - whi) if higher else (wlo - khi)   # clear space between the two ranges
            res[name] = {"white": round(w, 1), "black": round(k, 1),
                         "white_range": [round(wlo, 1), round(whi, 1)],
                         "black_range": [round(klo, 1), round(khi, 1)],
                         "threshold": round((w + k) / 2, 1), "line_reads_higher": higher}
            print("  {}: white {:.1f}% ({:.1f}-{:.1f}), black {:.1f}% ({:.1f}-{:.1f}), threshold {:.1f}%".format(
                name, w, wlo, whi, k, klo, khi, (w + k) / 2))
            if abs(k - w) < 10:
                print("  PROBLEM: white and black differ by under 10%. Check the A0 wire, and move the sensor closer to the floor.")
            elif gap <= 0:
                print("  WARNING: the white and black readings overlap. Hold the sensor steadier, or lower it.")
        else:
            k, fb = _sample_digital(read)
            res[name] = {"white": w, "black": k, "flicker": fw + fb}
            if w == k:
                print("  PROBLEM: {} reads {} on both. Check its wire, height and potentiometer.".format(name, w))
            elif fw + fb:
                print("  {} flickered {} times: turn its potentiometer away from the switching point.".format(name, fw + fb))
            else:
                print("  {}: white {}, black {}".format(name, w, k))
    if IR_MODE == "analog":
        ways = {v["line_reads_higher"] for v in res.values() if abs(v["black"] - v["white"]) >= 10}
        if ways == {True}:
            pol = "line reads higher than white"
        elif ways == {False}:
            pol = "line reads lower than white"
        else:
            pol = "unclear: fix the IR tracking sensors flagged above and run again"
    else:
        blacks = {v["black"] for v in res.values() if v["white"] != v["black"]}
        if blacks == {1}:
            pol = "1 = line (black)"
        elif blacks == {0}:
            pol = "0 = line (black)"
        else:
            pol = "unclear: fix the IR tracking sensors flagged above and run again"
    print("\nPolarity:", pol)
    R["ir"] = res
    R["ir_mode"] = IR_MODE
    R["ir_polarity"] = pol
    R["ir_sensors_fitted"] = [n for n, _ in IR_PINS]
    _save()


# ---------- 6. Ultrasonic ranging sensor ----------

def _ultra():
    return Pin(TRIG_PIN, Pin.OUT, value=0), Pin(ECHO_PIN, Pin.IN, Pin.PULL_DOWN)


def _ping(trig, echo, timeout_us=30000):
    trig.value(0)
    time.sleep_us(5)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    return time_pulse_us(echo, 1, timeout_us)   # negative on timeout


def sonar_live():
    """Show the Ultrasonic ranging sensor distance until you press Enter."""
    trig, echo = _ultra()
    _flush()
    print("Press Enter to stop.")
    while not _key():
        t = _ping(trig, echo)
        s = "no echo" if t < 0 else "{:6.1f} cm".format(t * CM_PER_US / 2)
        print("\r  " + s + "      ", end="")
        time.sleep_ms(100)
    print()


def sonar(distances=(10, 20, 30)):
    """Compare measured and real distances to a flat object."""
    if _ask_choice("Is Echo wired to GP18 through a 1:2 voltage divider?", ("y", "n")) == "n":
        print("  A 5 V Echo straight into GP18 can damage the Pico. Fit the divider first.")
        return
    trig, echo = _ultra()
    print("Use a flat object such as a book. Measure from the front face of the Ultrasonic ranging sensor.")
    rows = []
    for d in distances:
        input("\nPlace the object at {} cm, then press Enter...".format(d))
        vals = []
        misses = 0
        for _ in range(9):
            t = _ping(trig, echo)
            if t > 0:
                vals.append(t * CM_PER_US / 2)
            else:
                misses += 1
            time.sleep_ms(60)
        if not vals:
            print("  No echo at all. Check VCC (5 V), GND, Trig and Echo wiring.")
            rows.append({"actual_cm": d, "measured_cm": None, "missed": misses})
            continue
        vals.sort()
        med = vals[len(vals) // 2]
        print("  measured {:.1f} cm (spread {:.1f} cm), missed {} of 9".format(med, vals[-1] - vals[0], misses))
        rows.append({"actual_cm": d, "measured_cm": round(med, 1),
                     "spread_cm": round(vals[-1] - vals[0], 1), "missed": misses})
    R["sonar"] = rows
    _save()


# ---------- 7. battery ----------

def battery(secs=6):
    """Pack voltage at rest and with both motors running (multimeter across the pack)."""
    m = _need_map()
    if not m:
        return
    print("Multimeter on DC volts, probes on the battery pack's red and black wires.")
    idle = _ask_float("Pack voltage with the motors off (V)?")
    print("Lift the robot. Both motors run at full power for {} s: read the meter while they run.".format(secs))
    input("Press Enter to start...")
    try:
        _duty(m["left"]["forward_pin"], 1.0)
        _duty(m["right"]["forward_pin"], 1.0)
        time.sleep(secs)
    finally:
        stop()
    load = _ask_float("Pack voltage while the motors ran (V)?")
    R["battery_v"] = {"idle": idle, "motors_full_lifted": load}
    _save()


# ---------- 8. ruler measurements ----------

def measure():
    """Type in the ruler measurements of the robot and the track."""
    print("Use a ruler. Measure to the centers of things.")
    R["line_width_mm"] = _ask_float("Line width (mm)?")
    R["tightest_curve_radius_cm"] = _ask_float("Tightest curve radius on the track (cm)?")
    R["junction_types"] = input("Junction types on the track (for example: +, T, none)? ").strip()
    R["sensor_spacing_mm"] = _ask_float("Spacing between neighbouring IR tracking sensors, center to center (mm)?")
    R["sensor_height_mm"] = _ask_float("Height of the IR tracking sensors above the floor (mm)?")
    R["sensor_ahead_of_axle_mm"] = _ask_float("Distance from the wheel axle forward to the IR tracking sensors (mm)?")
    if "wheel_track_mm" not in R:
        R["wheel_track_mm"] = _ask_float("Distance between the centers of the two drive wheels (mm)?")
    R["wheel_diameter_mm"] = _ask_float("Drive wheel diameter (mm)?")
    _save()


# ---------- quick manual drive ----------

def _go(channel, speed):
    p1, p2 = channel
    speed = max(-1.0, min(1.0, speed))
    if speed >= 0:
        _duty(p2, 0)
        _duty(p1, speed)
    else:
        _duty(p1, 0)
        _duty(p2, -speed)


def _hold(secs):
    if secs:
        try:
            time.sleep(secs)
        finally:
            stop()


def r(speed=1.0, secs=1.0):
    """Right wheel (Motor A, GP0/GP1). b.r(1) one way, b.r(-1) the other way,
    b.r(0.5, 3) half power for 3 s. secs=0 keeps it running until b.stop()."""
    _go(CHANNEL_A, speed)
    _hold(secs)


def l(speed=1.0, secs=1.0):
    """Left wheel (Motor B, GP2/GP3). Same as b.r()."""
    _go(CHANNEL_B, speed)
    _hold(secs)


def both(left=1.0, right=1.0, secs=1.0):
    """Both wheels at once. b.both(1, 1) both one way, b.both(-1, -1) both the other way."""
    _go(CHANNEL_B, left)
    _go(CHANNEL_A, right)
    _hold(secs)


# ---------- summary ----------

_LABELS = (
    ("firmware", "MicroPython firmware"),
    ("board", "Board"),
    ("bootsel_button", "rp2.bootsel_button() available"),
    ("onboard_led", "Onboard LED"),
    ("pins", "Motor input test"),
    ("motor_map", "Motor map"),
    ("deadband", "Deadband, fraction of full duty"),
    ("trim", "Trim (scale factors)"),
    ("straight_drift_cm", "Last straight-line drift"),
    ("speed_cm_s", "Speed (cm/s) at duty"),
    ("ir_sensors_fitted", "IR tracking sensors fitted"),
    ("ir_mode", "IR tracking sensor mode"),
    ("ir", "IR tracking sensor readings"),
    ("ir_polarity", "IR tracking sensor polarity"),
    ("sonar", "Ultrasonic ranging sensor accuracy"),
    ("battery_v", "Battery pack voltage (V)"),
    ("line_width_mm", "Line width (mm)"),
    ("tightest_curve_radius_cm", "Tightest curve radius (cm)"),
    ("junction_types", "Junction types"),
    ("sensor_spacing_mm", "IR tracking sensor spacing (mm)"),
    ("sensor_height_mm", "IR tracking sensor height (mm)"),
    ("sensor_ahead_of_axle_mm", "IR tracking sensors ahead of axle (mm)"),
    ("wheel_track_mm", "Wheel track, center to center (mm)"),
    ("wheel_diameter_mm", "Wheel diameter (mm)"),
)


def _fmt(key, v):
    if v is None:
        return "not measured"
    if key == "motor_map":
        return "; ".join("{} wheel = Motor({}, {}, invert={})".format(
            w, v[w]["pin_a"], v[w]["pin_b"], v[w]["invert"]) for w in ("right", "left") if w in v)
    if key == "sonar":
        return "; ".join("{} cm reads {}".format(r["actual_cm"], r["measured_cm"]) for r in v)
    if key == "ir":
        return "; ".join(
            "{}: white {}%, black {}%, threshold {}%".format(n, x["white"], x["black"], x["threshold"])
            if "threshold" in x else
            "{}: white {}, black {}, flicker {}".format(n, x["white"], x["black"], x["flicker"])
            for n, x in v.items())
    if isinstance(v, dict):
        return ", ".join("{} {}".format(k, x) for k, x in v.items())
    if isinstance(v, list):
        return ", ".join(str(x) for x in v)
    return str(v)


def summary():
    """Print every result as a Markdown table and write it to results.md."""
    lines = ["| Measurement | Value |", "| --- | --- |"]
    for key, label in _LABELS:
        lines.append("| {} | {} |".format(label, _fmt(key, R.get(key))))
    text = "\n".join(lines)
    print(text)
    try:
        with open(SUMMARY_FILE, "w") as f:
            f.write(text + "\n")
        print("\nWritten to", SUMMARY_FILE)
    except OSError:
        pass


def help():
    print("""Run these in order:
  b.info()        firmware version, LED, BOOTSEL support
  b.pins()        which wheel each GP0-GP3 drives, and which way   (wheels lifted)
  b.deadband()    lowest power that turns each wheel                (on the floor)
  b.straight()    drive straight at 80%, measure drift, work out trim (floor, about 1.5 m clear)
  b.straight(0.9) the same at another power, for speed vs power
  b.ir_live()     live IR tracking sensor readings (analog %, or 0/1 in digital mode)
  b.ir()          each IR tracking sensor over white and over black; sets the line threshold
  b.sonar_live()  live Ultrasonic ranging sensor distance
  b.sonar()       Ultrasonic ranging sensor accuracy at 10, 20, 30 cm
  b.battery()     pack voltage at rest and under load               (wheels lifted, multimeter)
  b.measure()     ruler measurements of the robot and the track
  b.summary()     print everything as a Markdown table
  b.stop()        motors off

Move a wheel yourself (speed -1 to 1, then seconds; 0 seconds = keep running):
  b.r(1)          right wheel one way for 1 s     b.r(-1)  the other way
  b.l(1)          left wheel one way for 1 s      b.l(-1)  the other way
  b.both(1, 1)    both wheels, (left, right)      b.r(0.5, 3)  half power, 3 s""")


_load()
stop()
print("Bring-up tool loaded. Type b.help()")
