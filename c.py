from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.sensor import INPUT_1
import time
##m.on_for_rotations(75, 75, 100)
print("ready")
cs = ColorSensor(INPUT_1)
while True:
    time.sleep(1)
    print(cs.color)
    
    
        
        
