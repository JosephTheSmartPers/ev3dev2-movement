from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sensor import INPUT_4
import time
##m.on_for_rotations(75, 75, 100)
print("ready")
us = UltrasonicSensor(INPUT_4)
while True:
    time.sleep(1)
    print(us.distance_centimeters)
    
    
        
        
