from ev3dev2.sensor.lego import * 
from ev3dev2.sensor import *
gs = GyroSensor(INPUT_2)
print(gs.angle)
gs.reset()
gs.calibrate()
while True:
    print(gs.angle)
    time.sleep(1)