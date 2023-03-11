from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_1
import time
from ev3dev2.motor import *
##m.on_for_rotations(75, 75, 100)
us = UltrasonicSensor(INPUT_1)
leftm = LargeMotor(OUTPUT_A)
rightm = LargeMotor(OUTPUT_B)
m = MoveTank(OUTPUT_A, OUTPUT_B)
print("ready")
print(us.distance_centimeters)
m.on(75, 75)
while True:
    if (us.distance_centimeters <= 4.0):
        m.stop()
        print(us.distance_centimeters)
        print((leftm.rotations + rightm.rotations) / 2)
        break
