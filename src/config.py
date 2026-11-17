# All pins and tuning values for the lab4 line follower.
# Every other file takes its numbers from here.

# ---- Motors: L9110S (measured with bringup b.pins()) ----
#   GP0 -> A-1A   GP1 -> A-1B   left wheel   (forward = PWM on GP1)
#   GP2 -> B-1A   GP3 -> B-1B   right wheel  (forward = PWM on GP3)
LEFT_PINS = (0, 1)
RIGHT_PINS = (2, 3)
LEFT_INVERT = True
RIGHT_INVERT = True
PWM_FREQ = 1000

# Lowest duty that turns each wheel on the floor (bringup b.deadband()).
# Every non-zero command starts here, so these set the slowest possible speed.
# Measured on worn batteries: LOWER both if the robot is too fast even with FAST small.
LEFT_DEADBAND = 0.64
RIGHT_DEADBAND = 0.48

# ---- Line sensors: HW-870 (TCRT5000) modules, A0 outputs ----
#   GP26 <- left HW-870 A0     GP28 <- right HW-870 A0
IR_LEFT_PIN = 26
IR_RIGHT_PIN = 28
IR_AUTO_SIDES = True     # work out left/right from the start-up sweep (fixes swapped wires)
IR_LINE_READS = "auto"   # "auto", or force "high" / "low" if A0 over black is known

# ---- Obstacle sensor: HC-SR04 ----
#   GP19 -> Trig     GP18 <- Echo (through the 1:2 divider)
OBSTACLE_ON = True
TRIG_PIN = 19
ECHO_PIN = 18
STOP_CM = 15             # stop when something is nearer than this
CLEAR_CM = 25            # drive on once it is farther than this
PING_EVERY_MS = 100
PING_TIMEOUT_US = 6000   # about 1 m; no echo within this counts as clear

# ---- Speeds: 0 to 1, added on top of the deadband ----
FAST = 0.30              # base speed on straights
SLOW = 0.10              # base speed in the sharpest curves
CURVE_ERR = 0.6          # error at which the base speed is down to SLOW
TURN_MAX = 0.6           # largest steering correction
WHEEL_MIN = 0.0          # lowest wheel command while following (0 = inner wheel may stop)

# ---- Steering (PD) ----
# Error runs from -1 (line under the right sensor only) to +1 (left sensor only).
KP = 0.6
KD = 0.02                # per (error per second); 0 = steer on KP alone

# ---- Losing the line ----
# The start-up sweep finds out whether the line is wide enough to cover both sensors.
LOST_LEVEL = 0.15        # both sensors below this darkness = no line in view
# Wide line (covers both sensors, e.g. 19 mm tape): no line in view always means lost.
SIDE_ERR = 0.1           # an error this big is remembered as "the line is on that side"
BLIND_MS = 30            # no line for this long = lost: search toward the remembered side
# Narrow line (fits between the sensors): no line in view is normal when centred.
LOST_ERR = 0.3           # an error this big means the line was under one sensor
RECENT_MS = 400          # no line this soon after such an error = drove off: search
SEARCH_SPEED = 0.15      # pivot speed while searching
LOST_MS = 2000           # search this long, then stop and wait for the line

# ---- Start-up ----
START_DELAY_MS = 2000    # time to let go of the robot after power-on
CAL_SPEED = 0.10         # spin speed while sweeping the sensors across the line
CAL_LEFT_MS = 300        # sweep left at least this long...
CAL_SETTLE_MS = 120      # ...and until the readings have stopped changing for this long
CAL_MAX_MS = 2000        # ...but never longer than this. Then right for twice as long.
MIN_CONTRAST = 0.10      # white and black must differ by this fraction of 3.3 V
CAL_FILE = "cal.json"    # last good calibration, used if a sweep fails

# ---- Loop ----
LOOP_MS = 10
DEBUG = False            # True: print state, sensors and wheel commands 5 times a second
