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
wheelDiameter = 17.401
#? Settinng the base position of the robot to 0, and the angle also to 0 (0 meaning it is facing the longer side of the table opposite to the launch area)
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
def distance(x1, y1, x2, y2):
    return sqrt((x2 - x1) **2 + (y2 - y1) ** 2)
def getPointOnBezier(controllPoints, t):
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
    return binomial * t1 * t2
def factorial(n):
    if(n <= 1): return 1
    
    result = 1
    for i in range(2, n + 1):
        result *= i
    return result
def bezierLenght(controlPoints, n = 100):
        totalLength = 0
        t = 0
        dt = 1 / n
        point = controlPoints[0]
        previousPoint = getPointOnBezier(controlPoints, 0)
        
        for i in range(0, n):
            point = getPointOnBezier(controlPoints, t)
            totalLength += distance(previousPoint.x, previousPoint.y, point.x, point.y)
            print(degrees(atan2(point.y - previousPoint.y, point.x - previousPoint.x)))
            t += dt
            previousPoint = point
        return totalLength
        totalLength = 0
        t = 0
        dt = 1 / n
        point = controlPoints[0]
        previousPoint = getPointOnBezier(controlPoints, 0)
        
        for i in range(0, n):
            point = getPointOnBezier(controlPoints, t)
            totalLength += distance(previousPoint.x, previousPoint.y, point.x, point.y)
            print(degrees(atan2(point.y - previousPoint.y, point.x - previousPoint.x)))
            t += dt
            previousPoint = point
        return totalLength
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
    return round(float(returnSpeed), 2)
#* Functions that don't directly move the robot
def moveRobotOnLine(motor, szinSzenzor, minFeny, maxSebesseg, KP):
    returnSpeed = (minFeny - szinSzenzor.reflected_light_intensity) * KP
    #& Egyik oldali motor sebességének kiszámolása, egy célérték (minLight) és egy érzékenység (KP) alapján
    if(abs(returnSpeed) > maxSebesseg):            
        returnSpeed = maxSebesseg * (returnSpeed / abs(returnSpeed))
    #* Semmiképp se legyen az sebesség magasabb a megadott maximum sebességnél
    motor.on(returnSpeed)
    #* Elindítja a motort a kiszámolt sebességgel
    return round(float(returnSpeed), 2)
    #* Visszaadja a sebességet, hogy meg lehesen nézni hogy, 0, és mindkét motornak 0 lett a sebessége, akkor leáll a program.
def goOnLine(KP, maxIdo, maxSebesseg, minimumFeny):
    """Makes to robot perpendicularly go on a line, works with 2 sensors only."""
    elozoIdo = time()
    #? Vonalra állás kezdetének időpotját lementi
    while True:            
        elteltIdo = time() - elozoIdo
        #* Fordulás óta eltelt idő
        if(moveRobotOnLine(leftMotor, leftSensor, minimumFeny, maxSebesseg, KP) == 0 and moveRobotOnLine(rightMotor, rightSensor, minimumFeny, maxSebesseg, KP) == 0):
            tankMovement.stop(None, False)
            break
        #* Elindítja a motorokat a funkciókkal és megnézi, hogy mindkettő 0
        #* ha igen akkor leállítja a programot, mert elivleg sikeresen ráállt a vonalra
        if(elteltIdo >= maxIdo):
            tankMovement.stop(None, False)
            break
def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, correctMargin = 0, calibrate = False, speedingUp = False, slowingDown = False, debug = False):
    """Make the robot go straight in a specified degree (cm)"""
    global currentX
    global currentY
    global rotations
    startRotations = getRotations()
    timesGood = 0
    previousAngle = 0
    timesBad = 0
    if(speedingUp == False):
        speedingUp = distance * 0.5 * (abs(maxSpeed - minSpeed) / 99)
        if(speedingUp > (2 * wheelDiameter)):
            speedingUp = (2 * wheelDiameter)
    if(slowingDown == False):
        slowingDown = distance * 0.6
        if(slowingDown > (2*wheelDiameter)):
            slowingDown = (2 * wheelDiameter)
    direction = sign(maxSpeed)
    minSpeed *= direction
    tankMovement.on(minSpeed, minSpeed)
    while abs(getRotations() - startRotations) <= distance:
        if(calibrate == False):
            currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
            currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
            rotations = getRotations()
        if(stopOnLine == True or calibrate != False):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #* Ha be vonalraállás benne van a paraméterekben, és talál egy vonalat akkor megáll
                if(goOnLine == True):
                    goOnLine(1.75, 1.5, 15, 6)
                    #* Ha ponotsan vonalra állás be van kapcsolva akkor elindítja azt az eljárást.
                tankMovement.stop(None, False)
                break
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance, shouldSpeedUp = shouldSpeedUp)
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni
        sensitivityMultiplier = (calculatedSpeed / (maxSpeed - minSpeed)) * 2
        if(sensitivityMultiplier > 2):
            sensitivityMultiplier = 2
        if(sensitivityMultiplier < 0.5):
            sensitivityMultiplier = 0.5
        calculatedSensitivity = optimizeFloat(sensitivity / sensitivityMultiplier)
        if(abs(gsAngle() - targetAngle) > 0):
            calculatedAngle = ((gsAngle()) - targetAngle + drift) * calculatedSensitivity 
            #~     gyro célérték     jelenlegi gyro érték * érzékenység
            calculatedAngle *= direction
            #* Ne forduljon meg a robot hátra menésnél
            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            #* Ne tudjon a maximumnál nagyobb értékkel fordulni
            steeringMovement.on(calculatedAngle, calculatedSpeed)
            #* Elindítja a motort a kiszámolt sebességel és szögben.
            previousAngle = calculatedAngle
            timesBad += 1
            timesGood = 0
        else:
            if(timesBad + correctMargin != timesGood):
                steeringMovement.on((-previousAngle * direction), calculatedSpeed)
            else:
                steeringMovement.on((drift * direction), calculatedSpeed)
            timesGood += 1
        if(debug == True):
            print("Current Rotations: " + str(round(getRotations(), 2)) + "	Target Rotations: " + str(distance))
            print("Sensitivity: " + str(sensitivityMultiplier))
    newWheelDiameter = optimizeFloat(calibrate / ((abs(abs(getRotations()) - abs(startRotations)))/ wheelDiameter))
    if(motorStop):
        tankMovement.on(motorStop, motorStop)
    else:
        tankMovement.stop(None, False)
    if(calibrate):
        leftMotor.reset()
        rightMotor.reset()
        return (newWheelDiameter)
def gotoXY(targetX, targetY, maxSpeed, minSpeed, sensitvity, margin = 4, speedUp =0.3, slowDown = 0.8, debug = False, motorStop = False, shouldSlow = True, shouldSpeed = True, curve = False):
    """Goes to the specified coordinates (distance from the two sides in cm) using trigonometry"""
    global currentX
    global currentY
    global rotations
    rotations = getRotations()
    distance = (abs(abs(targetX - (margin)) - abs(currentX))) + abs((abs(targetY - (margin)) - abs(currentY)))
    startDistance = distance
    startGsAngle = gsAngle()
    speedUp *= startDistance 
    slowDown *= startDistance 
    startX = currentX
    startY = currentY
    if(maxSpeed < 0): minSpeed *= -1
    if(curve == True):
        coordinates = findBigger(abs(targetX - currentX), abs(targetY - currentY))
        curveX = -(coordinates[0] * sign(targetX - currentX))
        curveY = -(coordinates[1] * sign(targetY - currentY))
        curveTargetAngle = (((degrees(atan2((targetX - (targetX - curveX)), (targetY - (targetY -curveY))))))) 
        print("Curve Target Angle: " + str(curveTargetAngle))
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = (abs(abs(targetX - (margin)) - abs(currentX))) + abs((abs(targetY - (margin)) - abs(currentY)))
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()
        if(curve == True):
            if(curveY == 0):
                distanceDecimal = (1- abs((targetY - currentY) / (targetY - startY)))
                print("TargetY: " + str(targetY) + "	CurrentY: " + str(currentY))
            if(curveX == 0):
                distanceDecimal = (1 - abs((targetX - currentX) / (targetX - startX)))
                print("TargetX: " + str(targetX) + "	CurrentX: " + str(currentX)) 
            targetAngle = curveTargetAngle * distanceDecimal
            targetAngle += startGsAngle
            print(distanceDecimal)
        else:
            targetAngle = degrees(atan2(-(targetX - currentX), targetY - currentY))
        #targetAngle = shortest_angle(gsAngle(), targetAngle)
        if(maxSpeed < 0):
            targetAngle = ( targetAngle - 180)
        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance, shouldSpeedUp = shouldSpeed)
        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(calculatedSpeed) / calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        if(debug == True):
            print("----------------------------")
            print("X: "+str(round(currentX, 2)) + "	Y: " + str(round(currentY, 2)))
            print("targetX: "+str(targetX) + "	targetY: " + str(targetY))
            #print("Current Angle: " + str(gsAngle()) + "	Target Angle: " + str(round(targetAngle, 2)))
            #print("Left speed: " + str(leftm.speed) + "	Right speed: " + str(rightm.speed))
            print("slowDwon: "+str(slowDown) + "	speedUp: "+str(speedUp))
            print("Predicted Distance: " + str(distance) + "	Start Distance: " + str(startDistance))
            print("Target: " + str(targetAngle) + "	Calculated: " + str(calculatedAngle) + "	Current: " + str(gsAngle()))

        calculatedAngle = fmod(calculatedAngle, 360)
        calculatedAngle /= 3.6
        #* Ne forduljon meg a robot hátra menésnél

        if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
        steeringMovement.on(calculatedAngle, calculatedSpeed)
        sleep(0.01)

    if(motorStop == False):
        tankMovement.stop()
    else:
        tankMovement.on(motorStop, motorStop)
    print("__________")
    print("D__O__N__E")
    print("__________")
def fordul(angle, maxSpeed, sensitvity, relative = True, stopMargin = 2, minSpeed = 2):
    angle = angle * -1
    #? Így megy a jó irányba, gyro meg van fordítva
    fordulatszam = 0
    hasStopped = False
    if(relative == True):
        fordulatszam = gsAngle()
    turnConstant = (fordulatszam-angle)
    stopMargin *= sign(turnConstant)
    while gsAngle() != fordulatszam - angle:
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()
        if(turnConstant - stopMargin <= gsAngle() <= turnConstant + stopMargin and hasStopped == False and stopMargin != 0):
            hasStopped = True
            tankMovement.stop()
            print(str(turnConstant - stopMargin) + " | " + str(gsAngle()))
            continue
        calculatedSpeed = (turnConstant - gsAngle()) * sensitvity
        if(calculatedSpeed > maxSpeed / 3.5):
            calculatedSpeed = maxSpeed
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(maxSpeed) * sign(calculatedSpeed))
        if(abs(calculatedSpeed) < minSpeed): calculatedSpeed = (abs(minSpeed) * sign(calculatedSpeed))
        tankMovement.on(-calculatedSpeed, calculatedSpeed)
    tankMovement.stop()
def bezier(controllPoints, minSpeed, maxSpeed, sensitvity, margin=4, speedUp=0.3, slowDown = 0.8, motorStop=False, shouldSpeed= True, shouldSlow = True):
    targetX = controllPoints[len(controllPoints)-1].x
    targetY = controllPoints[len(controllPoints)-1].y
    global currentX
    global currentY
    global rotations
    rotations = getRotations()
    startRotations = rotations
    startGsAngle = gsAngle()
    startX = currentX
    startY = currentY
    distance = bezierLenght(controllPoints, 100)
    startDistance = distance
    print(distance)
    speedUp *= startDistance 
    slowDown *= startDistance

    previousPoint = getPointOnBezier(controllPoints, 0)
    previousDecimal = -1
    distanceDecimal = 0
    
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = getRotations() - startRotations
        distanceDecimal = (distance / startDistance)
        if(distanceDecimal > 1):
            break
        print("Decimal: " + str(distanceDecimal))
        
        currentPoint = getPointOnBezier(controllPoints, distanceDecimal)
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        print("TargetX: " + str(targetX) + "	CurrentX: " + str(currentX))
        print("TargetY: " + str(targetY) + "	CurrentY: " + str(currentY))
        rotations = getRotations()
        if(previousDecimal < distanceDecimal):
            targetAngle = -degrees(atan2(currentPoint.y - previousPoint.y, currentPoint.x - previousPoint.x))
            targetAngle -= startGsAngle
            targetAngle *= -1
            
        #targetAngle = shortest_angle(gsAngle(), targetAngle)
        print("Target angle: " + str(targetAngle))
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        print("Calculated angle: " + str(calculatedAngle))
        print("___")
        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance, shouldSpeedUp = shouldSpeed)
        if(sign(maxSpeed) != sign(calculatedAngle)):
            calculatedAngle = -calculatedAngle
        #* Ne forduljon meg a robot hátra menésnél
        if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
        steeringMovement.on((calculatedAngle / 3.6), calculatedSpeed)
        previousPoint = currentPoint
        previousDecimal = distanceDecimal
        sleep(0.01)
    if(motorStop == False):
        tankMovement.stop()
    else:
        tankMovement.on(motorStop, motorStop)
#* Functions having to do with movement of the robot
def futas1():
    print("Wheel diameter: " + str(wheelDiameter))
    #setPosition(-44, 24, 19)
    setPosition(-44, 24, 18.25)
    print(gyroOffset)
    print("X: " + str(currentX) + "	Y: " + str(currentY))
    #gotoXY(38, 40, 50.00, 6, 0.7, 5, 0.30, 0.7, debug=True)
    straight(31, 50, -37, 1.1, 5, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(80, 0.2)
    xHand.on_for_rotations(80, -0.65)
    yHand.on_for_rotations(80, 0.6, block=False)
    straight(5, 20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    xHand.on_for_rotations(80, -0.35, block=False)
    yHand.on_for_rotations(80, 1)
    straight(0.8, -20, -42, 1.1, 5, False, False, False, True, True, 0, 0)
    yHand.on_for_rotations(90, -2.22, block=False)
    
    straight(24, -40, -40, 1.1, 5, False, False, False, True, True, 0, 0)
    xHand.on_for_rotations(10, 1, block=False)
    yHand.on_for_rotations(10, 1.5, block=False)
    print(currentX)
    print(currentY)
    print(gsAngle())
    gotoXY(-42, 60, 50.00, 5.00, 1, 4, 0.30, 0.7, curve=True, debug=False, motorStop=50)
    xHand.on_for_rotations(-10, 1.5, block=False)
    straight(20, 50, 0, 1.1, 5, False, False, False, True, True, 0, 0)
    #gotoXY(35, 25, -50.00, 6, 0.7, 5, 0.30, 0.7, debug=True)
    #gotoXY(58, 44, 50.00, 6, 0.7, 5, 0.30, 0.7, debug=True)
    #gotoXY(58 / 2, 44 / 2, -50.00, 5.00, 0.8, 4, 0.30, 0.7, debug=False)
def run():
    bezier(targetX=469,targetY=151,controllPoints=[{x:483,y:238},{x:483,y:357},{x:469,y:357},{x:469,y:151}],minSpeed=10,maxSpeed=70,sens=0.8,speedUp=0.3,slowDown=0.8)
    bezier(targetX=691,targetY=80,controllPoints=[{x:469,y:151},{x:469,y:226.5},{x:691,y:226.5},{x:691,y:80}],minSpeed=10,maxSpeed=70,sens=0.8,speedUp=0.3,slowDown=0.8)