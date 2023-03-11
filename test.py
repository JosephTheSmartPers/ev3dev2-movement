from ev3dev2.sensor.lego import GyroSensor
from time import sleep
from ev3dev2.motor import MoveTank, MediumMotor
#? Mindent beimportálunk

yKez = MediumMotor("outD")
xKez = MediumMotor("outA")

yKez.on_for_rotations(50, 0.5)
