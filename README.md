# Lab 4 line follower

A Raspberry Pi Pico (MicroPython) robot that follows a black line on its own. Put it on the line and switch it on: after 2 s it sweeps its sensors across the line to calibrate, then follows it.

What it does:

- Steers smoothly from the two analog HW-870 line sensors and slows down in curves.
- Turns back toward the line if it runs off, and stops after 2 s if it can't find it. Put it back on a line and it carries on.
- Stops for an obstacle nearer than 15 cm and carries on once it's gone (needs the HC-SR04 powered, see Power).
- Goes straight through crossings.
- BOOTSEL button pauses and resumes.

| Onboard LED | Meaning |
| --- | --- |
| Steady | Following (also on during the start delay and calibration) |
| Slow blink | Lost the line, or paused |
| Fast blink | Obstacle ahead, or calibration failed: put it on the line and press BOOTSEL |
| Off | Program stopped |

## Wiring

| Pico pin | Connects to |
| --- | --- |
| GP0 (1), GP1 (2) | L9110S A-1A, A-1B: left wheel |
| GP2 (4), GP3 (5) | L9110S B-1A, B-1B: right wheel |
| GND (18) | Ground rail: L9110S GND, battery − |
| GP26 (31) | Left HW-870: A0 |
| GP28 (34) | Right HW-870: A0 |
| 3V3(OUT) (36), GND (28) | Both HW-870: VCC, GND |
| GP19 (25) | HC-SR04: Trig |
| GP18 (24) | HC-SR04: Echo, through the 1:2 divider |
| GND (23) | HC-SR04: GND |
| VBUS (40) | HC-SR04: VCC |
| VSYS (39) | LD1117V33 OUT, through a pull-out jumper (below) |

Battery + → switch → power rail → L9110S VCC.

## Power: running without USB

LD1117V33 ("LD33V"), printed side facing you, legs down:

| Leg | Pin | Connect to |
| --- | --- | --- |
| Left | GND | Ground rail |
| Middle | OUT | VSYS (pin 39), through a jumper you can pull out |
| Right | IN | Power rail (battery + after the switch) |

- 100 µF capacitor from OUT to GND, long leg to OUT. A 104 capacitor from IN to GND.
- The metal tab is OUT: keep it from touching anything.
- **Pull the OUT → VSYS jumper before plugging in USB.** Push it back in to run on battery.
- The HC-SR04 is powered from VBUS, which only has power on USB. On battery the robot gets no echoes and simply doesn't stop for obstacles. To try it on battery, move HC-SR04 VCC to the 3V3 rail and run `make hw-sonar`; some HC-SR04 boards work at 3.3 V, some don't.

## First run

Do these in order, USB plugged in, regulator jumper out, battery switch on.

1. `make test` runs the logic and simulation tests on the PC. No robot needed.
2. `make hw-motors` (wheels lifted). It must go forward, backward, spin left, spin right. If a wheel turns the wrong way, flip its `LEFT_INVERT` or `RIGHT_INVERT` in `src/config.py`.
3. `make hw-calibrate` with the robot on the line. It spins left, then right, and prints the levels it found. Then slide it by hand: each sensor should read about 0.00 over white and 1.00 over the tape.
4. `make run` with the robot on the line follows it, tethered. Ctrl-C stops it.
5. `make deploy`, unplug USB, push the regulator jumper in, put the robot on the line, switch on.

Other checks: `make hw-sensors` (raw sensor readings), `make hw-sonar` (HC-SR04).

## Tuning

Everything is in `src/config.py`. Set `DEBUG = True` to print the state, sensors and wheel commands while tethered.

| What you see | Change |
| --- | --- |
| Hums but doesn't move, or one wheel doesn't turn | Raise `FAST`, or that wheel's `*_DEADBAND` |
| Too fast, flies off curves | Lower `FAST` and `SLOW`. If it's still too fast at `FAST = 0.1`, lower both `*_DEADBAND` values |
| Wobbles on straights | Lower `KP` (try 0.4), or raise `KD` (try 0.04) |
| Slow to react, drifts wide in curves | Raise `KP` (try 0.8) |
| Steers away from the line | Set `IR_AUTO_SIDES = False` and swap `IR_LEFT_PIN` and `IR_RIGHT_PIN` |
| Calibration fails (fast blink) | Start on the line. If it doesn't spin, raise `CAL_SPEED`. If the line is faint, lower `MIN_CONTRAST` |
| Follows the floor, not the line | Set `IR_LINE_READS` to `"high"` (or `"low"`) |
| Spins too far at start-up | Lower `CAL_SPEED` and `CAL_LEFT_MS` |
| Spins too fast when searching | Lower `SEARCH_SPEED` |
| Stops for things that aren't there | Set `OBSTACLE_ON = False`, then check the HC-SR04 with `make hw-sonar` |

## Files

| File | Job |
| --- | --- |
| `src/config.py` | Every pin and tuning value |
| `src/main.py` | Entry point: starts the robot |
| `src/robot.py` | Builds the parts, runs calibration and the control loop |
| `src/follower.py` | Calibration maths, steering, lost-line and obstacle rules. No hardware, so it's tested on the PC |
| `src/car.py`, `src/motor.py` | Two-wheel drive on the L9110S, with deadband |
| `src/linesensor.py` | The two HW-870 sensors (A0) |
| `src/ultrasonic.py` | HC-SR04 |
| `tests/host/` | PC tests: logic, and the real code driving a simulated robot |
| `tests/hw/` | One small on-robot check per part |
| `bringup/bringup.py` | The measurement tool (`b.r()`, `b.l()`, `b.both()` move single wheels) |

## How it steers

Each sensor reading becomes a darkness $n$ from 0 (white) to 1 (line). The error is $e = n_L - n_R$, positive when the line is to the left.

$$\Delta = K_p\,e + K_d\,\frac{de}{dt}, \qquad u_L = u_{\text{base}} - \Delta, \qquad u_R = u_{\text{base}} + \Delta$$

$$u_{\text{base}} = \text{FAST} - (\text{FAST} - \text{SLOW})\,\min\!\left(1, \frac{|e|}{\text{CURVE\_ERR}}\right)$$

The start-up sweep also works out which sensor is on which side, which way A0 reads over black, and whether the line is wide enough to cover both sensors.

## Limits

- Tested in simulation only, not yet on the real robot. Expect to tune speeds and gains.
- The deadbands (0.64 left, 0.48 right) were measured on worn batteries; fresh ones will make it faster.
- With two sensors it can't tell a crossing from a straight, so it always goes straight through.
- At a sharp 90° corner it overshoots, then pivots back onto the line.
- Use black tape about 19 mm wide. A line narrower than the gap between the sensors works, but the robot can't tell when such a line ends.
