from math import sin, cos, degrees, atan2, radians
from time import time, sleep
from ev3dev2.sensor.lego import * 
from ev3dev2.sensor import *
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, MoveTank, MoveSteering

leftm = LargeMotor(OUTPUT_B)
rightm = LargeMotor(OUTPUT_C)

m = MoveTank(OUTPUT_B, OUTPUT_C)
s = MoveSteering(OUTPUT_B, OUTPUT_C)

gs = GyroSensor(INPUT_2)

rightSensor = ColorSensor("in3")
leftSensor = ColorSensor("in4")

m.on_for_rotations(30, 30, 2)
m.on_for_rotations(-30, -30, 2)