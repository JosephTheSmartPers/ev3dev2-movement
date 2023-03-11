from ev3dev2.sensor.lego import * 
from ev3dev2.sensor import *
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_A, SpeedPercent, MoveTank
m = MoveTank(OUTPUT_B, OUTPUT_A)
gs = GyroSensor(INPUT_2)
gs.reset()
gs.calibrate()
while True:
    print(gs.angle)
    time.sleep(1)