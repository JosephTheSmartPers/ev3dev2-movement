from math import sin, cos, degrees, atan2, radians
from time import time, sleep
from ev3dev2.sensor.lego import * 
from ev3dev2.sensor import *
from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C, MoveTank, MoveSteering

leftm = LargeMotor(OUTPUT_B)
rightm = LargeMotor(OUTPUT_C)

m = MoveTank(OUTPUT_B, OUTPUT_C)
s = MoveSteering(OUTPUT_B, OUTPUT_C)

gs = GyroSensor(INPUT_2)

rightSensor = ColorSensor("in3")
leftSensor = ColorSensor("in4")

gyroOffset = 0.99174

def gsAngle():
    return gs.angle * gyroOffset

def findBigger(num1, num2):
    """Subtracts the smallest number from all numbers in a vector and returns it."""
    smallest = min(num1, num2)
    num1 -= smallest
    num2 -= smallest
    return [num1, num2]

def getRotations():
    """Returns the current rotation of the two large motors"""
    return (leftm.rotations + rightm.rotations) / 2 * wheelDiameter

def sign(num):
    """Returns the sign of a number, and returns a 1 if the number is 0"""
    if(num == 0):
        return 1
    return(num / abs(num))

def shortest_angle(angle, target_angle):
    # Calculate the absolute difference between the angles
    diff = target_angle - angle
    
    # Check if the difference is more than 180 degrees
    if abs(diff) > 180:
        # Recalculate the target angle with the shortest path
        target_angle -= 360
        target_angle *= sign(diff) 
            
    return target_angle

currentX = 0
currentY = 0


def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance, shouldSpeedUp):
    """This function calculates the speed, for linearly speeding up and slowing down."""

    deltaDistance = abs(abs(currentDistance) - startDistance)
    #? Calculates how close the robot is to the target

    if(deltaDistance < speedUp and shouldSpeedUp == True):
            #* Ha a kezdet óta a fordulatok száma még kisebb annál a cél-fordulat számnál amit megadtunk, akkor tovább gyorsul
            return (deltaDistance / speedUp * (maxSpeed - minSpeed)) + minSpeed
            #~   [              0 és 1 közötti szám           ]   maximum elérhető érték (nem számítjuk a minimum sebességet) + alap sebesség

    elif(deltaDistance > distance - slowDown and shouldSlow == True):
        if(motorStop != False):
            minSpeed = motorStop
        #* Ha ez be van kapcsolva, akkor csak egy adott sebességig lassul, és utána bekapcsolva hagyja a motort
        return maxSpeed - ((deltaDistance - (distance - slowDown)) / slowDown * maxSpeed) + minSpeed
        #~               [                        1 és 0 közötti szám                      ]    legalacsonyabb sebessége a minimum érték lehet

    else:
        return maxSpeed

def raallSzog(motor, szinSzenzor, minFeny, maxSebesseg, KP):
    sebesseg = (minFeny - szinSzenzor.reflected_light_intensity) * KP
    #& Egyik oldali motor sebességének kiszámolása, egy célérték (minLight) és egy érzékenység (KP) alapján

    if(abs(sebesseg) > maxSebesseg):            
        sebesseg = maxSebesseg * (sebesseg / abs(sebesseg))
    #* Semmiképp se legyen az sebesség magasabb a megadott maximum sebességnél

    motor.on(sebesseg)
    #* Elindítja a motort a kiszámolt sebességgel

    return sebesseg
    #* Visszaadja a sebességet, hogy meg lehesen nézni hogy, 0, és mindkét motornak 0 lett a sebessége, akkor leáll a program.

def raall(KP, maxIdo, maxSebesseg, minimumFeny):
    elozoIdo = time()
    #? Vonalra állás kezdetének időpotját lementi

    while True:            
        elteltIdo = time() - elozoIdo
        #* Fordulás óta eltelt idő

        if(raallSzog(leftm, leftSensor, minimumFeny, maxSebesseg, KP) == 0 and raallSzog(rightm, rightSensor, minimumFeny, maxSebesseg, KP) == 0):
            m.stop(None, False)
            break
        #* Elindítja a motorokat a funkciókkal és megnézi, hogy mindkettő 0
        #* ha igen akkor leállítja a programot, mert elivleg sikeresen ráállt a vonalra

        if(elteltIdo >= maxIdo):
            m.stop(None, False)
            break
        #*


def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, correctMargin = 0, calibrate = False, speedingUp = False, slowingDown = False, debug = False):
    #Make the robot go staright in a specified degree (cm)

    startRotations = getRotations()
    timesGood = 0
    previousAngle = 0
    timesBad = 0
    status = "Starting"

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

    m.on(minSpeed, minSpeed)
    while abs(getRotations() - startRotations) <= distance:
        if(stopOnLine == True or calibrate != False):
             if(leftSensor.reflected_light_intensity <= 8 or rightSensor.reflected_light_intensity <= 8):
                #* Ha be vonalraállás benne van a paraméterekben, és talál egy vonalat akkor megáll
                if(goOnLine == True):
                    raall(1.75, 1.5, 15, 6)
                    #* Ha ponotsan vonalra állás be van kapcsolva akkor elindítja azt az eljárást.
                m.stop(None, False)
                break
             
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance, shouldSpeedUp = shouldSpeedUp)

        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen

        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni

        sensitivityMultiplier = (calculatedSpeed / (maxSpeed - minSpeed)) * 2

        if(sensitivityMultiplier > 2):
            sensitivityMultiplier = 2
        if(sensitivityMultiplier < 0.5):
            sensitivityMultiplier = 0.5

        calculatedSensitivity = sensitivity / sensitivityMultiplier
        

        if(abs(gsAngle() - targetAngle) > 0):
            calculatedAngle = ((gsAngle()) - targetAngle + drift) * calculatedSensitivity 
            #~     gyro célérték     jelenlegi gyro érték * érzékenység

            calculatedAngle *= direction
            #* Ne forduljon meg a robot hátra menésnél

            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            #* Ne tudjon a maximumnál nagyobb értékkel fordulni

            """if(gsAngle() == irany):
                pontos += 1
            osszesMeres += 1"""
            #* Pontosságot számolja

            s.on(calculatedAngle, calculatedSpeed)
            #* Elindítja a motort a kiszámolt sebességel és szögben.
            previousAngle = calculatedAngle
            timesBad += 1
            timesGood = 0
        else:
            if(timesBad + correctMargin != timesGood):
                s.on((-previousAngle * direction), calculatedSpeed)
            else:
                s.on((drift * direction), calculatedSpeed)
            timesGood += 1
        if(debug == True):
            print("Current Rotations: " + str(round(getRotations(), 2)) + "\tTarget Rotations: " + str(distance))
            print(sensitivityMultiplier)
    newWheelDiameter = (calibrate / ((abs(abs(getRotations()) - abs(startRotations)))/ wheelDiameter))
    if(motorStop != False):
        m.on(motorStop, motorStop)
    else:
        m.stop(None, False)
    if(calibrate):
        leftm.reset()
        rightm.reset()
        return (newWheelDiameter)

def gotoXY(targetX, targetY, maxSpeed, minSpeed, sensitvity, margin = 4, speedUp =0.3, slowDown = 0.8, debug = False, motorStop = False, shouldSlow = True, shouldSpeed = True, curve = False):
    global currentX
    global currentY
    global rotations
    rotations = getRotations()

    distance = (abs(abs(targetX) - abs(currentX))) + abs((abs(targetY) - abs(currentY)))

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
        print("Curve target: " + str(curveTargetAngle))

    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        status = "Starting"

        distance = (abs(abs(targetX) - abs(currentX))) + abs((abs(targetY) - abs(currentY)))

        currentX += (sin(radians(gsAngle())) * ((getRotations() - rotations))) * -1
        currentY += cos(radians(gsAngle())) * ((getRotations() - rotations))
        rotations = getRotations()

        if(curve == True):
            if(curveY == 0):
                distanceDecimal = abs(1- abs((targetY - currentY) / (targetY - startY)))
                print("TargetY: " + str(targetY) + "\tCurrentY: " + str(currentY))
            if(curveX == 0):
                distanceDecimal = abs(1 - abs((targetX - currentX) / (targetX - startX)))
                print("TargetX: " + str(targetX) + "\tCurrentX: " + str(currentX))
                  
            #distanceDecimal = 1- (abs(abs(abs(abs(targetX) - abs(curveX)) - abs(currentX)) + abs(abs(abs(targetY) - abs(curveY)) - abs(currentY))) / startCurveDistance)
            print("Decimal: " + str(distanceDecimal))
            targetAngle = curveTargetAngle * distanceDecimal
        else:
            targetAngle = degrees(atan2(targetX - currentX, targetY - currentY))

        targetAngle += startGsAngle

        print("Target angle: " + str(targetAngle))

        

        targetAngle = shortest_angle(gsAngle(), targetAngle)

        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance, shouldSpeedUp = shouldSpeed)

        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(calculatedSpeed) / calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni

        print("Equation: " + str(gsAngle()) + "+" + str(targetAngle))

        calculatedAngle = ((gsAngle()) - targetAngle) * sensitvity
        print("Calculated angle: " + str(calculatedAngle) +" \n")


        if(debug == True):
            print("----------------------------")
            print("X: "+str(round(currentX, 2)) + "\tY: " + str(round(currentY, 2)))
            print("targetX: "+str(targetX) + "\ttargetY: " + str(targetY))
            print("Current Angle: " + str(gsAngle()) + "\tTarget Angle: " + str(round(targetAngle, 2)))
            print("Left speed: " + str(leftm.speed) + "\tRight speed: " + str(rightm.speed))
            #print("slowDwon: "+str(slowDown) + "\tspeedUp: "+str(speedUp))
            #print("Predicted Distance: " + str(distance) + "\tStart Distance: " + str(startDistance))
            #print("Status: " + status)
            print("Target: " + str(targetAngle) + "\tCalculated: " + str(calculatedAngle) + "\tCurrent: " + str(gsAngle()))

        
        
        #~     gyro célérték     jelenlegi gyro érték * érzékenység

        if(maxSpeed < 0):
            calculatedAngle = calculatedAngle
        #* Ne forduljon meg a robot hátra menésnél

        if(abs(calculatedAngle) > 100): calculatedAngle = (abs(calculatedAngle) / calculatedAngle) * 100
        s.on(calculatedAngle, calculatedSpeed)
        sleep(0.01)
    if(motorStop == False):
        m.stop()
    else:
        m.on(motorStop, motorStop)

    print("__________")
    print("D__O__N__E")
    print("__________")

dist = 83
wheelDiameter = 17.50439367311072
def calibrate():
    gs.reset()
    global wheelDiameter
    wheelDiameter = 1
    wheelDiameter = straight(10, 35, 0, 1.15, 5, False, False, False, True, True, -1, 0, calibrate=float(dist))

    sleep(0.1)
    straight(float(dist) + 2, -50, 0, 1.1, 5, False, False, False, True, True, 0, 0)



    print(wheelDiameter)
#calibrate()

wheelDiameter = float(wheelDiameter)

#straight(100, 50, 0, 1.1, 5, False, False, False, True, True, 0, 0)
rotations = getRotations()
input("Start? ")

currentX = 40
currentY = 40
gs.reset()
leftm.reset()
rightm.reset()
try:
    gotoXY(120, 90, 50.00, 5.00, 0.8, 4, 0.30, 0.7, curve=True, debug=False, motorStop=50)
    gotoXY(150, 55, 50.00, 5.00, 1, 4, 0.30, 0.7, curve=True, debug=False, shouldSpeed= False, motorStop=50)
    gotoXY(37.5, 40.5, 50.00, 5.00, 0.8, 2, 0.30, 0.7, curve=True, debug=False, shouldSpeed= False)
    bestSong = "https://youtube.com/shorts/MS2ZXbbZp3o?feature=share"
    #gotoXY(40, 40, 20, 5.00, 0.8, 1.5, 0.30, 0.7, True)

except KeyboardInterrupt:
    m.stop()
    m.reset()
    print("Exited the program")
m.stop()
m.reset()
    
