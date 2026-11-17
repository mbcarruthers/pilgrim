# HC-SR04 check, using the same quick ping the robot uses. Motors stay off.
#   make hw-sonar
# Hold a book in front: it should print the distance. "no echo" beyond about 1 m,
# or always if the HC-SR04 has no 5 V (it is powered from VBUS: USB only).
# Ctrl-C to stop.

import time
import config
from ultrasonic import Ultrasonic
from follower import Obstacle

sonar = Ultrasonic(config.TRIG_PIN, config.ECHO_PIN)
obstacle = Obstacle(config.STOP_CM, config.CLEAR_CM)
while True:
    cm = sonar.ping_cm(config.PING_TIMEOUT_US)
    blocked = obstacle.update(cm)
    print("{}   {}".format("no echo" if cm is None else "{:5.1f} cm".format(cm),
                           "STOP" if blocked else "clear"))
    time.sleep_ms(200)
