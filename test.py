#!/usr/bin/env micropython
from math import sin, cos, degrees, atan2, radians, sqrt, fmod
from time import time, sleep
from ev3dev2.sensor.lego import GyroSensor, ColorSensor
from ev3dev2.motor import LargeMotor, MoveTank, MoveSteering, MediumMotor
from ev3dev2.power import PowerSupply
from ev3dev2.console import Console
from ev3dev2.button import Button
from os import system
from ev3dev2.led import Leds
from ev3dev2.sound import Sound
#? Imports
speaker = Sound()
#console = Console()
#console.set_font(font='Lat15-Terminus32x16', reset_console=False)
button = Button()
leds = Leds()
battery = PowerSupply()
leftMotor = LargeMotor("outB")
rightMotor = LargeMotor("outC")
#? Individual motors defined
tankMovement = MoveTank(leftMotor.address, rightMotor.address)
steeringMovement = MoveSteering(leftMotor.address, rightMotor.address)
#? Getting the adresses of previously set motors and defining the movements based on those.
yHand = MediumMotor("outD")
xHand = MediumMotor("outA")
#? Defining the Medium Motors operating the vertical motion of the robot, and the extensions
gyro = GyroSensor("in2")
rightSensor = ColorSensor("in3")
leftSensor = ColorSensor("in4")
gs = GyroSensor("in2")
gs.mode = GyroSensor.MODE_GYRO_ANG

gyroCorrection = 0.99174
#? The gyroscope of the robot counts 363 degrees in one rotation rather than 360, so we always multiply it by this constant
gyroOffset = 0
currentX = 0
currentY = 0
rotations = 0
rotationsText = open("rotations.txt", "r+")
wheelDiameter = float(rotationsText.readline())
rotationsText.close()
wheelDiameter = 17.5

def optimizeFloat(num):
    return round(float(num), 4)
def sign(num):
    """Returns the sign of a number, and returns a 1 if the number is 0"""
    if(num == 0):
        return 1
    return(num / abs(num))
def findBigger(num1, num2):
    """Subtracts the smallest number from all numbers in a vector and returns it."""
    smallest = min(num1, num2)
    num1 -= smallest
    num2 -= smallest
    return [num1, num2]
#* Functions that are only mathematical, and don't have any direct connection to the robot

def gsAngle():
    """Returns the angle of the gyroscope, (this also takes into account the offset and the correction of the gyroscope)"""
    return fmod(optimizeFloat((gyro.angle + gyroOffset) * gyroCorrection), 360)
def getRotations():
    """Returns the current rotation of the two large motors"""
    return optimizeFloat((leftMotor.rotations + rightMotor.rotations) / 2 * wheelDiameter)
def setPosition(angle, x, y):
    """Sets the gyroscope angle and the x and y coordinates of the robot."""
    global gyroOffset
    global currentX
    global currentY
    gyroOffset = angle
    currentX = x
    currentY = y
def shortest_angle(angle, target_angle):
    # Calculate the absolute difference between the angles
    diff = target_angle - angle
    # Check if the difference is more than 180 degrees
    if abs(diff) > 180:
        # Recalculate the target angle with the shortest path
        target_angle -= 360
        target_angle *= sign(diff) 
    return target_angle
def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance, shouldSpeedUp):
    """This function calculates the speed, for linearly speeding up and slowing down."""
    deltaDistance = abs(abs(currentDistance) - startDistance)
    #? Calculates how close the robot is to the target
    returnSpeed = maxSpeed
    
    if(deltaDistance < speedUp and shouldSpeedUp == True):
        #* Ha a kezdet óta a fordulatok száma még kisebb annál a cél-fordulat számnál amit megadtunk, akkor tovább gyorsul
        returnSpeed = (deltaDistance / speedUp * (maxSpeed - minSpeed)) + minSpeed
        #~   [              0 és 1 közötti szám           ]   maximum elérhető érték (nem számítjuk a minimum sebességet) + alap sebesség
    elif(deltaDistance > distance - slowDown and shouldSlow == True):
        if(motorStop != False):
            minSpeed = motorStop
        #* Ha ez be van kapcsolva, akkor csak egy adott sebességig lassul, és utána bekapcsolva hagyja a motort
        returnSpeed = maxSpeed - ((deltaDistance - (distance - slowDown)) / slowDown * maxSpeed) + minSpeed
        #~               [                        1 és 0 közötti szám                      ]    legalacsonyabb sebessége a minimum érték lehet
    if(abs(returnSpeed) > 100): returnSpeed = sign(returnSpeed) * 100
    
    return round(float(returnSpeed), 2)
#* Functions that don't directly move the robot
def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, correctMargin = 0, calibrate = False, speedingUp = False, slowingDown = False, debug = False, timeout = False):
    """Make the robot go straight in a specified degree (cm)"""
    direction = sign(maxSpeed)
    startTime = time()
    minSpeed *= direction
    if(shouldSpeedUp == True):
        tankMovement.on(minSpeed, minSpeed)
    else:
        tankMovement.on(maxSpeed, maxSpeed)
    global currentX
    global currentY
    global rotations
    startRotations = getRotations()
    timesBad = 0
    if(speedingUp == False):
        speedingUp = distance * 0.5 * (abs(maxSpeed - minSpeed) / 99)
        if(speedingUp > (2 * wheelDiameter)):
            speedingUp = (2 * wheelDiameter)
    if(slowingDown == False):
        slowingDown = distance * 0.6
        if(slowingDown > (2*wheelDiameter)):
            slowingDown = (2 * wheelDiameter)
    while abs(getRotations() - startRotations) <= distance:
        if(timeout and startTime + timeout <= time()):
            break
        if(calibrate == False):
            currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
            currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
            rotations = getRotations()
        if(stopOnLine == True):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #* Ha be vonalraállás benne van a paraméterekben, és talál egy vonalat akkor megáll
                if(goOnLine == True):
                    goOnLine(1.75, 1.5, 15, 6)
                    #* Ha ponotsan vonalra állás be van kapcsolva akkor elindítja azt az eljárást.
                tankMovement.stop(None, False)
                break
        if(calibrate):
             if(leftSensor.reflected_light_intensity <= 92 or rightSensor.reflected_light_intensity <= 92):
                #* Ha be vonalraállás benne van a paraméterekben, és talál egy vonalat akkor megáll
                if(goOnLine == True):
                    goOnLine(1.75, 1.5, 15, 6)
                    #* Ha ponotsan vonalra állás be van kapcsolva akkor elindítja azt az eljárást.
                tankMovement.stop(None, False)
                break
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance, shouldSpeedUp = shouldSpeedUp)
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni
        calculatedSensitivity = sensitivity
        if(abs(gsAngle() - targetAngle) > 0):
            calculatedAngle = ((gsAngle()) - targetAngle + drift) * calculatedSensitivity 
            #~     gyro célérték     jelenlegi gyro érték * érzékenység
            calculatedAngle *= direction
            #* Ne forduljon meg a robot hátra menésnél
            calculatedAngle /= 3.6
            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            #* Ne tudjon a maximumnál nagyobb értékkel fordulni
            steeringMovement.on(calculatedAngle, calculatedSpeed)
            #* Elindítja a motort a kiszámolt sebességel és szögben.
        """else:
            if(timesBad + correctMargin != timesGood):
                steeringMovement.on((-previousAngle * direction), calculatedSpeed)
            else:
                steeringMovement.on((drift * direction), calculatedSpeed)
            timesGood += 1
        if(debug == True):
            print("Current Rotations: " + str(round(getRotations(), 2)) + "\tTarget Rotations: " + str(distance))
            print("Sensitivity: " + str(sensitivityMultiplier))"""
    if(calibrate):
        newWheelDiameter = optimizeFloat(calibrate / ((abs(abs(getRotations()) - abs(startRotations)))/ wheelDiameter))
    if(motorStop):
        tankMovement.on(motorStop, motorStop)
    else:
        tankMovement.stop(None, False)
    if(calibrate):
        leftMotor.reset()
        rightMotor.reset()
        return (newWheelDiameter)
def meow():
    gs.reset()
    yHand.on_for_rotations(-100, 1, False, True)
    yHand.reset()
    yHand.on_for_rotations(70, 2.5, True)
    straight(100, 70, 0, 3, 25)
    xHand.on_for_rotations(-100, 1)
    xHand.on_for_rotations(100, 1, block = False)
    yHand.on_for_rotations(50, 2.5, True, False)
    straight(100, -100, 0, 3, 25, shouldSlowDown=False, shouldSpeedUp=False)
meow()