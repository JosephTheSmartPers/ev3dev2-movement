from ev3dev2.sensor.lego import * 
from ev3dev2.sensor import *
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_A, SpeedPercent, MoveTank, MoveSteering
from random import *
from ev3dev2.power import PowerSupply
import time 
from datetime import datetime
from ev3dev2.sound import Sound
m = MoveTank(OUTPUT_A, OUTPUT_B)
s = MoveSteering(OUTPUT_A, OUTPUT_B)
## cs = ColorSensor(INPUT_1)
##ps = ColorSensor(INPUT_2)
us = UltrasonicSensor(INPUT_1)
lus = UltrasonicSensor(INPUT_3) ##right
rus = UltrasonicSensor(INPUT_4) ##left
#cs = ColorSensor(INPUT_2)
gs = GyroSensor(INPUT_2)
leftm = LargeMotor(OUTPUT_A)
rightm = LargeMotor(OUTPUT_B)
spkr = Sound()
global normal
gs.reset()

gs.MODE_GYRO_ANG = 'GYRO-ANG'
normal = gs.angle 
def staright():
    normal = gs.angle
    m.on(75, 75)
    while True:
        if(gs.angle > normal):
            while gs.angle != normal:
              s.on(1, 75) 
            m.on(75, 75)
        elif(gs.angle < normal):
            while gs.angle != normal:
                s.on(-1, 75)
            m.on(75, 75) 

            
def turn(degree):
    global normal
    rldegree = degree * -1
    rotation = gs.angle
    while gs.angle != rotation - rldegree:
        if(rotation - rldegree < gs.angle):
            m.on(-75, 75)
        elif(rotation - rldegree < gs.angle):
            m.on(75, -75)
    normal = gs.angle  

    
def obsticle(dir = 1):
    turn(-90 * dir)
    
def fordul(irany = 1, tavolsag = 2):
    m.stop()
    spkr.speak('Get out of my way')
    m.on_for_rotations(-75, -75, 1)
    if(lus.distance_centimeters <= 10.0):
        obsticle()
    elif(rus.distance_centimeters <= 10.0):
        obsticle(-1)
    else:
        obsticle()
    if(us.distance_centimeters <= 10.0):
        turn(180)
        
        return
    if(us.distance_centimeters <= 10.0):
        m.on_for_rotations(-75, 75, 1.9)
        turn(90)
        
        while True:
            ##if(cs.color == 0):
                ##fordul() 
            if(us.distance_centimeters <= 10.0):
                m.on_for_rotations(-75, -75, 0.5)
                turn(-90)
                if(us.distance_centimeters <= 10.0):
                    turn(180)
                    print("corner")
                    

    m.on(75, 75)

print("ready")
m.on(75, 75)

current = (leftm.rotations + rightm.rotations) / 2
sensors = lus.distance_centimeters + rus.distance_centimeters + us.distance_centimeters
now = datetime.now()

time = now.strftime("%S")
while True:
    now = datetime.now()
    
    if(us.distance_centimeters <= 10.0):
        print("ultra")
        fordul(0.5)
        normal = gs.angle
    if(int(time) + 1 == int(now.strftime("%S"))):
        sensors = lus.distance_centimeters + rus.distance_centimeters + us.distance_centimeters
        if((sensors + 4) <= lus.distance_centimeters + rus.distance_centimeters + us.distance_centimeters >= sensors - 4):
            m.stop()
            m.on_for_degrees(-75, -75, 1)
            if(lus.distance_centimeters <= 10.0):
                obsticle()
            elif(rus.distance_centimeters <= 10.0):
                obsticle(-1)
            else:
                obsticle()
            m.on(75, 75)
        time = now.strftime("%S")

        if(gs.angle > normal):
            s.on(-1, 75)
            print("h")
            print(gs.angle)
            print(normal)
        elif(gs.angle < normal):
            s.on(1, 75)
            print("hi")
    """if(cs.color == 0):
       print("color")
       fordul()"""
    
        

        