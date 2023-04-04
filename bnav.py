from math import sin, cos, degrees, atan2, radians, sqrt
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
"""yHand = MediumMotor("outA")
xHand = MediumMotor("outD")"""
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
#? Settinng the base position of the robot to 0, and the angle also to 0 (0 meaning it is facing the longer side of the table opposite to the launch area)
class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y
def distance(x1, y1, x2, y2):
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2))
def getPointOnBezier(controllPoints, t):
    n = len(controllPoints) -1
    x = 0
    y = 0
    for i in range(0, n):
        bi = bernstein(n, i, t)
        x += controllPoints[i].x * bi
        y += controllPoints[i].y * bi
    return Vector(x, y)
def bernstein(n, i, t):
    if(i<0 or i > n): return 0
    if(n == 0 and i == 0): return 1

    binomial = factorial(n) / (factorial(i) * factorial(n -1))
    t1 = pow(1 -t, n - i)
    t2 = pow(t, i)
    return binomial * t1 * t2
def factorial(n):
    if(n <= 1): return 1
    
    result = 1
    for i in range(2, n):
        result *= i
    return result
def bezierLenght(controlPoints, n = 100):
        totalLength = 0
        t = 0
        dt = 1 / n
        point = controlPoints[0]
        
        for i in range(0, n):
            nextPoint = getPointOnBezier(controlPoints, t + dt)
            totalLength += distance(point.x, point.y, nextPoint.x, nextPoint.y)
            point = nextPoint
            t += dt
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
    return optimizeFloat((gyro.angle + gyroOffset) * gyroCorrection)
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
            print("Current Rotations: " + str(round(getRotations(), 2)) + "\tTarget Rotations: " + str(distance))
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
    if(curve == True):
        coordinates = findBigger(abs(targetX - currentX), abs(targetY - currentY))
        curveX = -(coordinates[0] * sign(targetX - currentX))
        curveY = -(coordinates[1] * sign(targetY - currentY))
        curveTargetAngle = -abs(-gsAngle()-((degrees(atan2((targetX - (targetX - curveX)), (targetY - (targetY -curveY))))))) 
        print(curveTargetAngle)
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = (abs(abs(targetX - (margin)) - abs(currentX))) + abs((abs(targetY - (margin)) - abs(currentY)))
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()
        if(curve == True):
            if(curveY == 0):
                distanceDecimal = abs(1- abs((targetY - currentY) / (targetY - startY)))
                print("TargetY: " + str(targetY) + "\tCurrentY: " + str(currentY))
            if(curveX == 0):
                distanceDecimal = abs(1 - abs((targetX - currentX) / (targetX - startX)))
                print("TargetX: " + str(targetX) + "\tCurrentX: " + str(currentX)) 
            targetAngle = curveTargetAngle * distanceDecimal
            targetAngle += startGsAngle
        else:
            targetAngle = degrees(atan2(-(targetX - currentX), targetY - currentY))
        targetAngle = shortest_angle(gsAngle(), targetAngle)
        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance, shouldSpeedUp = shouldSpeed)
        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(calculatedSpeed) / calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        if(debug == True):
            print("----------------------------")
            print("X: "+str(round(currentX, 2)) + "\tY: " + str(round(currentY, 2)))
            print("targetX: "+str(targetX) + "\ttargetY: " + str(targetY))
            #print("Current Angle: " + str(gsAngle()) + "\tTarget Angle: " + str(round(targetAngle, 2)))
            #print("Left speed: " + str(leftm.speed) + "\tRight speed: " + str(rightm.speed))
            print("slowDwon: "+str(slowDown) + "\tspeedUp: "+str(speedUp))
            print("Predicted Distance: " + str(distance) + "\tStart Distance: " + str(startDistance))
            #print("Target: " + str(targetAngle) + "\tCalculated: " + str(calculatedAngle) + "\tCurrent: " + str(gsAngle()))
        if(maxSpeed < 0):
            calculatedAngle = -calculatedAngle
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
    print("\nDONE\n")
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
    
    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        distance = getRotations() - startRotations
        distanceDecimal = (distance / startDistance)
        print("Decimal: " + str(distanceDecimal))
        currentPoint = getPointOnBezier(controllPoints, distanceDecimal)
        currentX -= sin(radians(gsAngle())) * ((getRotations() - rotations))
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()
        targetAngle = -degrees(atan2(currentPoint.y - previousPoint.x, currentPoint.x - previousPoint.x))
        targetAngle = shortest_angle(gsAngle(), targetAngle)
        print("Target angle: " + str(targetAngle))
        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        print("Calculated angle: " + str(calculatedAngle))
        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance, shouldSpeedUp = shouldSpeed)
        if(sign(maxSpeed) != sign(calculatedAngle)):
            calculatedAngle = -calculatedAngle
        #* Ne forduljon meg a robot hátra menésnél
        if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
        steeringMovement.on(calculatedAngle, calculatedSpeed)
        previousPoint = currentPoint
        sleep(0.01)
    if(motorStop == False):
        tankMovement.stop()
    else:
        tankMovement.on(motorStop, motorStop)


#* Functions having to do with movement of the robot
def calibrate():
    """Runs a short calibration script, that calculates how many cm is one rotation."""
    gyro.reset()
    global wheelDiameter
    wheelDiameter = 1
    wheelDiameter = straight(10, 35, 0, 1.15, 5, False, False, False, True, True, -1, 0, calibrate=float(dist))
    sleep(0.1)
    straight(float(dist) + 2, -50, 0, 1.1, 5, False, False, False, True, True, 0, 0)
    print(wheelDiameter)
def futas1():
    print("Wheel diameter: " + str(wheelDiameter))
    #setPosition(-44, 24, 19)
    setPosition(-44, 22, 14.25)
    print(gyroOffset)
    print("X: " + str(currentX) + "\tY: " + str(currentY))
    gotoXY(44, 44, 50.00, 6, 0.7, 5, 0.30, 0.7, debug=True)
    yHand.on_for_rotations(90, 0.4, False, False)
    xHand.on_for_rotations(90, -1)
    sleep(0.1)
    yHand.on_for_rotations(100, 1)
    sleep(0.1)
    yHand.on_for_rotations(90, -1)
    gotoXY(30, 30, -50.00, 6, 0.7, 5, 0.30, 0.7, debug=True)
    #gotoXY(58, 44, 50.00, 6, 0.7, 5, 0.30, 0.7, debug=True)
    #gotoXY(58 / 2, 44 / 2, -50.00, 5.00, 0.8, 4, 0.30, 0.7, debug=False)

dist = 83
wheelDiameter = 17.58164165931156
#calibrate()
wheelDiameter = float(wheelDiameter)
rotations = getRotations()
input("Start? ")
setPosition(0, 40, 40)
gyro.reset()
leftMotor.reset()
rightMotor.reset()
"""try:
    futas1()   
    print("meow")
except KeyboardInterrupt:
    tankMovement.stop()
    tankMovement.reset()
    print("Exited the program")
exit("this wont even work")"""
try:
    bezier([Vector(0, 0), Vector(25, 25), Vector(75, 25), Vector(75, 100)], 10, 70, 1, 4)
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