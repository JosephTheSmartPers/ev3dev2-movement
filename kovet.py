from ev3dev2.sensor.lego import * 
from ev3dev2.sensor import *
from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_B, SpeedPercent, MoveTank, MoveSteering
from random import *
import time
m = MoveTank(OUTPUT_B, OUTPUT_C)
s = MoveSteering(OUTPUT_B, OUTPUT_C)
cs = ColorSensor(INPUT_1)
m.on(-75, 75)
while True:
    r = cs.reflected_light_intensity
    if(r != 10):
        if(10 < r):
            
        s.on_for_rotations(-3, 75, 0.1)
        if(r > cs.reflected_light_intensity):
            while cs.reflected_light_intensity > 7:
                s.on(-3, 75)
            m.on(75, 75)
        elif(r < cs.reflected_light_intensity):
            while cs.reflected_light_intensity > 7:
                s.on(3, 75)
            m.on(75, 75)
            print(cs.reflected_light_intensity)
    


    
    
    