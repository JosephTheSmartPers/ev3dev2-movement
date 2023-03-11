from ev3dev2.sensor.lego import * 
from ev3dev2.sensor import *
from ev3dev2.motor import *
from random import *
import time
leftm = LargeMotor(OUTPUT_A)
rightm = LargeMotor(OUTPUT_D)
m = MoveTank(OUTPUT_A, OUTPUT_D)
s = MoveSteering(OUTPUT_A, OUTPUT_D)
cs = ColorSensor(INPUT_1)

def follow(spd, dist, target, sens = 1, opp = 1):
    while (rightm.position + leftm.position) / 2 <= dist:
        deg = (cs.reflected_light_intensity - target) * sens
        if(deg > 100):
            deg = 100
        elif deg < -100:
            deg = -100
        s.on(deg, spd)
    s.stop()
follow(30, 99999999999999999999999999999999999999999999999999999999, 45, 1.2)