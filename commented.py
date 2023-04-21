# Color coding
#? For explaining declarations.
#* For explaning classes and functions
#! Something that is important to look at later, or might be confusing
#~ For explaining very long and confusing calulcations.
#^ For explaining a chunk of code, or a group of functions
from math import sin, cos, degrees, atan2, radians, sqrt, fmod
from time import time, sleep
from ev3dev2.sensor.lego import GyroSensor, ColorSensor
from ev3dev2.motor import LargeMotor, MoveTank, MoveSteering, MediumMotor
#? Imports
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
#? Defining the sensors of the robot.
gyroCorrection = 0.99174
#? The gyroscope of the robot counts 363 degrees in one rotation rather than 360, so we always multiply it by this constant
gyroOffset = 0
currentX = 0
currentY = 0
wheelDiameter = 17.5198
#? Settinng the base position of the robot to 0, and the angle also to 0 (0 meaning it is facing the longer side of the table opposite to the launch area)
class Vector:
    """#* This class is used for the bezier curve, it's a simple 2D vector."""
    def __init__(self, x, y):
        self.x = -x
        self.y = y
def distance(x1, y1, x2, y2):
    """#* A basic distance calculatoion for a two dimentional vector, with the distance being calculated insid the function."""
    return sqrt((x2 - x1) **2 + (y2 - y1) ** 2)
def getPointOnBezier(controllPoints, t):
    """#* Returns the X and Y value of a single point on a bezier curve based on a decimal of [0, 1]"""
    n = len(controllPoints) -1
    x = 0
    y = 0
    for i in range(n + 1):
        bi = bernstein(n, i, t)
        x += controllPoints[i].x * bi
        y += controllPoints[i].y * bi
    return Vector(x, y)
def bernstein(n, i, t):
    if i < 0 or i > n:
        return 0
    if n == 0 and i == 0:
        return 1

    binomial = factorial(n) / (factorial(i) * factorial(n - i))
    t1 = pow(1 -t, n - i)
    t2 = pow(t, i)

    print(binomial * t1 * t2)
    return binomial * t1 * t2

def factorial(n):
    """#* Returns the factorial of a number"""
    if(n <= 1): return 1
    result = 1
    for i in range(2, n + 1):
        result *= i
    return result
def bezierLenght(controlPoints, n = 100):
        """#* Returns the lenght of a bezier curve based on controllpoints and the number of sections (n)"""
        totalLength = 0
        t = 0
        dt = 1 / n
        point = controlPoints[0]
        previousPoint = getPointOnBezier(controlPoints, 0)
        
        for _ in range(0, n):
            point = getPointOnBezier(controlPoints, t)
            totalLength += distance(previousPoint.x, previousPoint.y, point.x, point.y)
            t += dt
            previousPoint = point
        return totalLength
def optimizeFloat(num):
    """#* This function just rounds the float to a given value, so it uses less computing power as working with long doubles isn't as efficient"""
    return round(float(num), 4)
def sign(num):
    """#* Returns the sign of a number, and returns a 1 if the number is 0"""
    if(num == 0):
        return 1
    return(num / abs(num))
def findBigger(num1, num2):
    """#* Subtracts the smallest number from all numbers in a vector and returns it."""
    smallest = min(num1, num2)
    num1 -= smallest
    num2 -= smallest
    return [num1, num2]
#^ Functions that are only mathematical, and don't have any direct connection to the robot
def gsAngle():
    """#* Returns the angle of the gyroscope, (this also takes into account the offset and the correction of the gyroscope)"""
    return fmod(optimizeFloat((gyro.angle + gyroOffset) * gyroCorrection), 360)
def getRotations():
    """#* Returns the current rotation of the two large motors"""
    return optimizeFloat((leftMotor.rotations + rightMotor.rotations) / 2 * wheelDiameter)
def setPosition(angle, x, y):
    ""#* Sets the gyroscope angle and the x and y coordinates of the robot."""
    global gyroOffset
    global currentX
    global currentY
    gyroOffset = angle
    currentX = x
    currentY = y
def shortest_angle(angle, target_angle):
    """#* Calculates the shortest path the robot has the make to reach a desired angle."""
    diff = target_angle - angle
    # ?Calculate the absolute difference between the angles
    if abs(diff) > 180:
        #? Check if the difference is more than 180 degrees
        target_angle -= 360
        target_angle *= sign(diff) 
        #? Recalculate the target angle with the shortest path
    return target_angle
def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance, shouldSpeedUp):
    """#* This function calculates the speed at a given distance, for linearly speeding up and slowing down."""
    deltaDistance = abs(abs(currentDistance) - startDistance)
    #? Calculates how close the robot is to the target
    returnSpeed = maxSpeed
    if(deltaDistance < speedUp and shouldSpeedUp == True):
        #? If the motors rotations aren't higher than the specified speedUp value it will continue to accelerate the robot.
        returnSpeed = (deltaDistance / speedUp * (maxSpeed - minSpeed)) + minSpeed
        #~               a decimal of [0,1]        subtracting minSpeed because it will always go with atleast minSpeed
    elif(deltaDistance > distance - slowDown and shouldSlow == True):
        #? If the distance from the target is larger than the distance of the target in the beggining and the distance from which you should be slowing down, you start to slow down
        if(motorStop != False):
            minSpeed = motorStop
        #? If motorStop isn't false, the robot will only slow down to the provided percentage (and it will keep it's motors on after the function)
        returnSpeed = maxSpeed - ((deltaDistance - (distance - slowDown)) / slowDown * maxSpeed) + minSpeed
        #~                                         decimal of [0, 1]          inversse of the previous calculation of speeding up + minSpeed
    if(abs(returnSpeed) > 100): returnSpeed = sign(returnSpeed) * 100
    #? Checks if the returned percentage is larger than 100
    return round(float(returnSpeed), 2)
#^ Functions that don't directly move the robot
def moveRobotOnLine(motor, colorSensor, minLight, maxSpeed, sensitivity):
    """#* Moves the specified side of the robot based on that color sensors data."""
    returnSpeed = (minLight - colorSensor.reflected_light_intensity) * sensitivity
    #~ Calculating the speed of the motor based on the light of the Color sensor on the provided side of the robot.
    if(abs(returnSpeed) > maxSpeed):            
        returnSpeed = maxSpeed * (returnSpeed / abs(returnSpeed))
    #? Making sure calculated speed isn't larger than the provided maxSpeed
    motor.on(returnSpeed)
    return round(float(returnSpeed), 2)
    #? Returns the calculated speed (rounded because of computation) so the program knows when the speed is 0 so it knows the robot has found the line.
def goOnLine(sensitivity, timeout, maxSpeed, minLight):
    """#* Makes to robot perpendicularly go on a line, works with 2 sensors only."""
    startTime = time()
    while True:            
        if(moveRobotOnLine(leftMotor, leftSensor, minLight, maxSpeed, sensitivity) == 0 and moveRobotOnLine(rightMotor, rightSensor, minLight, maxSpeed, sensitivity) == 0):
            #? If the calculated speed for both motors are 0, the program stops.
            tankMovement.stop(None, False)
            break
        if(time() - startTime >= timeout):
            tankMovement.stop(None, False)
            break
def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, correctMargin = 0, calibrate = False, speedingUp = False, slowingDown = False, debug = False):
    """#* Make the robot go straight in a specified degree for a specified distance (cm)"""
    if(shouldSpeedUp == True):
        tankMovement.on(minSpeed, minSpeed)
    else:
        tankMovement.on(maxSpeed, maxSpeed)
    #? Starts the motors before some minor computations to save some time.
    global currentX
    global currentY
    global rotations
    startRotations = getRotations()
    timesGood = 0
    previousAngle = 0
    timesBad = 0
    if(speedingUp == False):
        speedingUp = distance * 0.3 * (abs(maxSpeed - minSpeed) / 99)
        if(speedingUp > (wheelDiameter)):
            speedingUp = (wheelDiameter)
    if(slowingDown == False):
        slowingDown = distance * 0.7
        if(slowingDown > (wheelDiameter)):
            slowingDown = (wheelDiameter)
    #? Sets the default values for speeding up and slowing down.
    direction = sign(maxSpeed)
    minSpeed *= direction
    while abs(getRotations() - startRotations) <= distance:
        if(calibrate == False):
            currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
            currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
            rotations = getRotations()
        #? If it tries to calibrate the rotations get reset it would go for a long time if it didn't find the line.
        if(stopOnLine == True or calibrate != False):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #? Checks if it should stop when it detects a line.
                if(goOnLine == True):
                    goOnLine(1.75, 1.5, 15, 6)
                    #? Checks if it should perpendiculalrly go on a line.
                tankMovement.stop(None, False)
                break
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance, shouldSpeedUp = shouldSpeedUp)
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        sensitivityMultiplier = (calculatedSpeed / (maxSpeed - minSpeed)) * 2
        if(sensitivityMultiplier > 2):
            sensitivityMultiplier = 2
        if(sensitivityMultiplier < 0.5):
            sensitivityMultiplier = 0.5
        calculatedSensitivity = optimizeFloat(sensitivity / sensitivityMultiplier)
        #? The sensitivity decreases as the robot goes faster.
        if(abs(gsAngle() - targetAngle) > 0):
            calculatedAngle = ((gsAngle()) - targetAngle + drift) * calculatedSensitivity 
            calculatedAngle *= direction
            calculatedAngle /= 3.6
            #? The movesteering only accepts a percentage rather than an angle.
            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            steeringMovement.on(calculatedAngle, calculatedSpeed)
            previousAngle = calculatedAngle
            timesBad += 1
            timesGood = 0
            #? Starts counting how many times it had to coorigate.
        else:
            if(timesBad + correctMargin != timesGood):
                steeringMovement.on((-previousAngle * direction), calculatedSpeed)
            else:
                steeringMovement.on((drift * direction), calculatedSpeed)
            timesGood += 1
            #? If it had to coorigate it goes the same distance in the other direction so it's closer to the original line it should be going on.
        if(debug == True):
            print("Current Rotations: " + str(round(getRotations(), 2)) + "\tTarget Rotations: " + str(distance))
            print("Sensitivity: " + str(sensitivityMultiplier))
    if(calibrate):
        newWheelDiameter = optimizeFloat(calibrate / ((abs(abs(getRotations()) - abs(startRotations)))/ wheelDiameter))
    if(motorStop):
        tankMovement.on(motorStop, motorStop)
    else:
        tankMovement.stop(None, False)
    #? If motorStop isn't false it makes the motors go at the specified speed for a seemless transition.
    if(calibrate):
        leftMotor.reset()
        rightMotor.reset()
        return (newWheelDiameter)
def gotoXY(targetX, targetY, maxSpeed, minSpeed, sensitvity, margin = 4, speedUp =0.3, slowDown = 0.8, debug = False, motorStop = False, shouldSlow = True, shouldSpeed = True, curve = False):
    """#* Goes to the specified coordinates (distance from the two sides in cm) using trigonometry"""
    global currentX
    global currentY
    global rotations
    rotations = getRotations()
    distance = (abs(abs(targetX - (margin)) - abs(currentX))) + abs((abs(targetY - (margin)) - abs(currentY)))
    #? Aproximating the distance from the target.
    startDistance = distance
    startGsAngle = gsAngle()
    speedUp *= startDistance 
    slowDown *= startDistance 
    startX = currentX
    startY = currentY
    if(maxSpeed < 0): minSpeed *= -1
    if(curve == True):
        coordinates = findBigger(abs(targetX - currentX), abs(targetY - currentY))
        #? Finds if x or y is smaller
        curveX = -(coordinates[0] * sign(targetX - currentX))
        curveY = -(coordinates[1] * sign(targetY - currentY))
        #? These values are what the delta distance from the coordinates should be after the robot has completed the turn.
        curveTargetAngle = (((degrees(atan2((targetX - (targetX - curveX)), (targetY - (targetY -curveY))))))) 
        #? It then calculates an angle it should go in when it has completed the curve to reach it's target.
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = (abs(abs(targetX - (margin)) - abs(currentX))) + abs((abs(targetY - (margin)) - abs(currentY)))
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()
        if(curve == True):
            if(curveY == 0):
                distanceDecimal = (1- abs((targetY - currentY) / (targetY - startY)))
                print("TargetY: " + str(targetY) + "\tCurrentY: " + str(currentY))
            if(curveX == 0):
                distanceDecimal = (1 - abs((targetX - currentX) / (targetX - startX)))
                print("TargetX: " + str(targetX) + "\tCurrentX: " + str(currentX)) 
            targetAngle = curveTargetAngle * distanceDecimal
            targetAngle += startGsAngle
            #? Calculates an angle based on how far the robot is from the nearer x or y position.
        else:
            targetAngle = degrees(atan2(-(targetX - currentX), targetY - currentY))
        #targetAngle = shortest_angle(gsAngle(), targetAngle)
        if(maxSpeed < 0):
            targetAngle = ( targetAngle - 180)
        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance, shouldSpeedUp = shouldSpeed)
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(calculatedSpeed) / calculatedSpeed) * abs(maxSpeed)
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        if(debug == True):
            print("----------------------------")
            print("X: "+str(round(currentX, 2)) + "\tY: " + str(round(currentY, 2)))
            print("targetX: "+str(targetX) + "\ttargetY: " + str(targetY))
            print("Current Angle: " + str(gsAngle()) + "\tTarget Angle: " + str(round(targetAngle, 2)))
            print("slowDwon: "+str(slowDown) + "\tspeedUp: "+str(speedUp))
            print("Predicted Distance: " + str(distance) + "\tStart Distance: " + str(startDistance))
            print("Target: " + str(targetAngle) + "\tCalculated: " + str(calculatedAngle) + "\tCurrent: " + str(gsAngle()))
        calculatedAngle = fmod(calculatedAngle, 360)
        calculatedAngle /= 3.6
        if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
        steeringMovement.on(calculatedAngle, calculatedSpeed)
        sleep(0.01)
    if(motorStop == False):
        tankMovement.stop()
    else:
        tankMovement.on(motorStop, motorStop)
def turn(targetAngle, maxSpeed, sensitvity, relative = True, stopMargin = 2, minSpeed = 2, timeout = 2):
    targetAngle = targetAngle * -1
    startingPoint = 0
    hasStopped = False
    if(relative == True):
        startingPoint = gsAngle()
    #? Wether it should turn to the targetAngle based on 0 or the current gyroscope angle.
    turnConstant = (startingPoint-targetAngle)
    stopMargin *= sign(turnConstant)
    startTime = time()
    while gsAngle() != startingPoint - targetAngle:
        if(time() - startTime > timeout): break
        if(turnConstant - stopMargin <= gsAngle() <= turnConstant + stopMargin and hasStopped == False and stopMargin != 0):
            #? Stops the robot completely for one cycle if it's near to the targetAngle.
            hasStopped = True
            tankMovement.stop()
            continue
        calculatedSpeed = (turnConstant - gsAngle()) * sensitvity
        if(calculatedSpeed > maxSpeed / 3.5):
            calculatedSpeed = maxSpeed
        #? This is just to make a robot go with maxSpeed for a little longer, which still won't make it overturn.
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(maxSpeed) * sign(calculatedSpeed))
        if(abs(calculatedSpeed) < minSpeed): calculatedSpeed = (abs(minSpeed) * sign(calculatedSpeed))
        tankMovement.on(-calculatedSpeed, calculatedSpeed)
    tankMovement.stop()
def bezier(controllPoints, minSpeed, maxSpeed, sensitvity, margin=4, speedUp=0.3, slowDown = 0.8, motorStop=False, shouldSpeed= True, shouldSlow = True):
    """#* Makes the robot drive any bezier curve, note that it's much easier to draw one and get the controllPoints with my editor."""
    targetX = controllPoints[len(controllPoints)-1].x
    targetY = controllPoints[len(controllPoints)-1].y
    global currentX
    global currentY
    global rotations
    rotations = getRotations()
    startRotations = rotations
    startGsAngle = gsAngle()
    distance = bezierLenght(controllPoints, 100)
    startDistance = distance
    previousPoint = getPointOnBezier(controllPoints, 0)
    previousDecimal = -1
    distanceDecimal = 0
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = getRotations() - startRotations
        distanceDecimal = (distance / startDistance)
        if(distanceDecimal > 1):
            break
        currentPoint = getPointOnBezier(controllPoints, distanceDecimal)
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()
        if(previousDecimal < distanceDecimal):
            targetAngle = -degrees(atan2(currentPoint.y - previousPoint.y, currentPoint.x - previousPoint.x))
            targetAngle -= 2*startGsAngle
            targetAngle *= -1
            #? Calculates the angle which the robot should go in based on the previous and the current points coordinates.
        targetAngle = shortest_angle(gsAngle(), targetAngle)
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        calculatedSpeed = calculateSpeed(distanceDecimal, 0, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distanceDecimal, shouldSpeedUp = shouldSpeed)
        if(sign(maxSpeed) == -1):
            calculatedAngle = -calculatedAngle
        if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
        steeringMovement.on((calculatedAngle / 3.6), calculatedSpeed)
        previousPoint = currentPoint
        previousDecimal = distanceDecimal
        sleep(0.01)
    if(motorStop == False):
        tankMovement.stop()
    else:
        tankMovement.on(motorStop, motorStop)
#^ Functions having to do with movement of the robot
def calibrate():
    """#* Runs a short calibration script, that calculates how many cm is one rotation."""
    gyro.reset()
    global wheelDiameter
    wheelDiameter = 1
    wheelDiameter = straight(10, 35, 0, 1.15, 5, False, False, False, True, True, 0, 0, calibrate=float(dist))
    sleep(0.1)
    straight(float(dist) + 2, -50, 0, 1.1, 5, False, False, False, True, True, 0, 0)
    print(wheelDiameter)
def run1():
    setPosition(-44, 24, 18.25)
    straight(30, 40, -40, 0.95, 5, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(80, 0.2)
    xHand.on_for_rotations(80, -0.45)
    #? Pushing the water cell away, so the normal cell can roll back to home.
    yHand.on_for_rotations(80, 0.6, block=False)
    straight(4.5, 20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(80, 1)
    #? Going into the dam and lifting the hand when it's just below the spinning part, so the green cell comes out.
    straight(0.8, -20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    xHand.on_for_rotations(45, -0.65, block=False)
    yHand.on_for_rotations(90, -2.22, block=False)
    #? Going back a bit and extending the arm fully so the robot hooks onto the water cell previously pushed away.
    straight(25, -40, -40, 1.1, 5, False, False, False, True, True, 0, 0)
    tankMovement.on_for_rotations(40,40,0.2)
    #? Coming back and going forward a bit so the operators can remove the water cell because the robot isn't touching it anymore
    xHand.on_for_rotations(15, 0.7, block=False)
    yHand.on_for_rotations(10, 2.5, block=False)
    bezier(controllPoints=[Vector(50, 36),Vector(61, 35),Vector(60, 64)],minSpeed=10,maxSpeed=40,sensitvity=0.98,speedUp=0.3,slowDown=0.8, motorStop=40)
    #? Going to the energy storage (the bezier only goes there about 3/4 because it has to move it's hand out of the way)
    xHand.on_for_rotations(-100, 1, block=False)
    straight(20, 40, 5, 1.2, 7, False, False, 35, False, True, 0, 0, slowingDown=0.1)
    #? Moving the arm and going to energy storage.
    steeringMovement.on_for_rotations(3, 35, 0.8)
    #? Going forwards a bit with no specified angle so the __RÁVEZETŐ__ works better.
    tankMovement.on_for_rotations(-20, -20, 0.05)
    yHand.on_for_rotations(-80, 2.5, block=False)
    #? Moves back a bit and then lowers the hand into the storage container.
    sleep(0.2)
    gyro.reset()
    setPosition(0, 180, 97)
    sleep(0.2)
    straight(distance=13,maxSpeed=-20,targetAngle=0,sensitivity=0.8,minSpeed=5,speedingUp=0.5,slowingDown=0.3)
    xHand.on_for_rotations(70, 0.55)
    #? Moves back a bit and moves the hand under the oil platform
    raiseSpeed = 70
    raiseHeight = 0.6
    yHand.reset()
    yHand.on_for_rotations(raiseSpeed, 1.2, block=True)
    sleep(0.1)
    yHand.on_for_rotations(-raiseSpeed, raiseHeight, block=True)
    sleep(0.01)
    for _ in range(0,2):
        yHand.on_for_rotations(raiseSpeed, raiseHeight, block=True)
        sleep(0.01)
        yHand.on_for_rotations(-raiseSpeed, raiseHeight, block=True)
        sleep(0.01)
    yHand.on_for_rotations(raiseSpeed, -(1.1), block=True)
    #? Lowers and raises its hand a total of 3 times to get the three oil cells out of their containtainers.
    xHand.on_for_rotations(-10, 0.5, False, False)
    turn(-85, 70, 0.38, False, 10, 2, 1)
    straight(distance=3.8,maxSpeed=20,targetAngle=-85,sensitivity=0.8,minSpeed=5,speedingUp=0.5,slowingDown=0.3)
    turn(-88, 70, 0.38, False, 10, 3, 1)
    #? Turns so it's back is facing the oil truck, this can only be done in two turns, because of lack of space.
    straight(distance=20,maxSpeed=-70,targetAngle=-90,sensitivity=0.8,minSpeed=5,speedingUp=0.5, shouldSlowDown=False)
    straight(distance=8,maxSpeed=20,targetAngle=-90,sensitivity=0.8,minSpeed=5,speedingUp=0.5, shouldSlowDown=False)
    turn(-45, 70, 0.38, False, 10, 3, 1)
    straight(distance=50,maxSpeed=-70,targetAngle=-25,sensitivity=0.8,minSpeed=5,speedingUp=0.5, shouldSlowDown=False)
    #? Rams it's back into the oil truck, so it goes in the home area, than comes back itself.   
def run2():
    yHand.on_for_rotations(30, 1.2, True, False)
    #? Moves the hand up as one gear would be in the way for the energy cells.
    setPosition(-29, 33.5, 19.5)
    straight(90, 70, -30, 4, 5)
    turn(-90, 30, 0.45, relative=False, timeout=0.6)
    straight(10, 40, -90, 5, 25, motorStop=30)
    #? Collects the 3 energy cells from the solar farm, than starts going to the connector.
    turn(-98, 30, 0.45, relative=False, timeout=0.6)
    yHand.on_for_rotations(-90, 1.2, True, False)
    straight(15, 40, -98, 5, 25)
    #? It has to face a bit more right compared to 90° for the __EXTENSION__ to work best.
    tankMovement.on_for_rotations(-30, -30, 0.15)
    #? Also has to move a bit back for the __EXTENSION__ to work best.
    yHand.on_for_rotations(100, 2.2, True, True)
    turn(-180, 30, 0.5, relative=False, timeout=0.6)
    #?Raises it's hand so it won't knock the 2 water cells over.
    tankMovement.on_for_rotations(-30, -30, 0.8)
    gyro.reset()
    setPosition(180, 106, 112)
    sleep(0.2)
    #? Rams into the wall so it can reset the gyroscope
    straight(2, 10, 180, 1, 25)
    straight(23, 50, 234, 2, 25, motorStop=20)
    straight(21, 30, 198, 10, 25, shouldSpeedUp=False)
    #? Collects the 2 water cells, and goes into the circe in the middle.
    straight(13, -30, int(gsAngle())+75, 3.6, 20)
    yHand.on_for_rotations(-70, 2.2, True, False)
    turn(207, 30, 0.6, False, 2, 0.6)
    #? Leaves the 3 power cells in the circle.
    straight(20, 15, 215, 6, 6, True, False)
    straight(9, 20, 215, 6, 1)
    #? Finds the line and then goes a bit further.
    turn(178, 60, 0.6, False, 2, 0.35)
    straight(20, 50, 178, 5, 10)
    straight(0.2, -30, 180, 5, 1)
    xHand.on_for_rotations(40, 2, True, True)
    #? Goes into the power plant than goes back a bit so it can get all the power cells out.
    straight(7.8, -30, 180, 5, 1)
    turn(270, 40, 0.25, False, 2, 0.8)
    straight(30, 100, 255, 5, 6, slowingDown=0.05, motorStop=100)
    straight(40, 100, 275, 5, 6, slowingDown=0.05, speedingUp=0.95, motorStop=100, shouldSpeedUp=False)
    #? Goes back a bit and goes into the other launch area.
def run3():
    setPosition(0, 200, 18)
    straight(24, 70, 0, 1.2, 20, motorStop=25)
    straight(15, 25, 0, 0.8, 15, shouldSpeedUp=False)
    straight(7, -60, 0, 1.2, 20)
    #? Pushes the couch into place than comes back a bit.
    turn(47, 40, 0.2, False, 2, 0.35)
    xHand.on_for_rotations(-40, 2, block=False)
    straight(45, 80, 47, 3.5, 10, motorStop=80)
    yHand.on_for_rotations(100, 0.5, block=False)
    straight(20, 80, 39, 6, 10, shouldSpeedUp=False)
    xHand.on_for_rotations(100, 2, block=False)
    #? Goes to the car platform and flips the lever.
def run4():
    gyro.reset()
    setPosition(80, 200, 3)
    yHand.on_for_rotations(50, 0.8)
    xHand.on_for_rotations(50, 0.1)
    straight(57, 80, 80, 3, 5, motorStop=60)
    straight(50, 60, 42, 4, 5, shouldSpeedUp=False)
    xHand.on_for_rotations(-50, 1.2)
rotations = getRotations()
dist = 83
wheelDiameter = 17.6232
#calibrate()
wheelDiameter = optimizeFloat(wheelDiameter)
input("Start? ")
setPosition(0, 40, 40)
gyro.reset()
leftMotor.reset()
rightMotor.reset()
try:
    run4()   
    print("meow")
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
    xHand.stop()
    yHand.stop()
    xHand.reset()
    yHand.reset()
    print("Exited the program")
tankMovement.stop()
tankMovement.reset()
xHand.stop()
yHand.stop()
xHand.reset()
yHand.reset()
exit("this wont even work")
controlPoints = [Vector(0, 0), Vector(25, 25), Vector(75, 25)]
setPosition(0, 0, 0)
controlPoints = [Vector(75, 25), Vector(25, 25), Vector(0, 0)]
setPosition(0, 75, 25)
try:
    bezier(controlPoints, 10, 70, 0.5, 4)
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
exit("meow")
try:
    gotoXY(120, 80, 50.00, 5.00, 1, 4, 0.30, 0.7, curve=True, debug=False, motorStop=50)
    gotoXY(150, 55, 50.00, 5.00, 1, 4, 0.30, 0.7, curve=True, debug=False, shouldSpeed= False, motorStop=50)
    gotoXY(37.5, 40.5, 50.00, 5.00, 0.8, 2, 0.30, 0.7, curve=True, debug=False, shouldSpeed= False)
    bestSong = "https://youtube.com/shorts/MS2ZXbbZp3o?feature=share"
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
    print("Exited the program")
tankMovement.stop()
tankMovement.reset()