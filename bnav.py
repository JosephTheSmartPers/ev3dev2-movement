import math
import time
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

currentX = 0
currentY = 0


def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance):
    """This function calculates the speed, for linearly speeding up and slowing down."""

    deltaDistance = abs(abs(currentDistance) - startDistance)
    #? Calculates how close the robot is to the target

    if(deltaDistance < speedUp):
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


def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSpeedUp = True, shouldSlowDown = True, drift = 0, margin = 0, calibrate = False, debug = False):
    #Make the robot go staright in a specified degree (cm)

    startRotations = getRotations()
    timesGood = 0
    previousAngle = 0
    timesBad = 0
    status = "Starting"
    speedingUp = distance * 0.5 * (abs(maxSpeed - minSpeed) / 99)
    slowingDown = distance * 0.6
    direction = sign(maxSpeed)
    minSpeed *= direction

    if(speedingUp > (2 * wheelDiameter)):
        speedingUp = (2 * wheelDiameter)
    if(slowingDown > (2*wheelDiameter)):
        slowingDown = (2 * wheelDiameter)


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
             
        calculatedSpeed = calculateSpeed(getRotations(), startRotations, speedingUp, slowingDown, maxSpeed, minSpeed, motorStop, shouldSlowDown, distance)

        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen

        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = sign(calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni

        sensitivityMultiplier = (calculatedSpeed / (maxSpeed - minSpeed)) * 2

        if(sensitivityMultiplier > 2):
            sensitivityMultiplier = 2
        if(sensitivityMultiplier < 0.5):
            sensitivityMultiplier = 0.5

        calculatedSensitivity = sensitivity / sensitivityMultiplier
        

        if(abs(gs.angle - targetAngle) > 0):
            calculatedAngle = ((gs.angle) - targetAngle + drift) * calculatedSensitivity 
            #~     gyro célérték     jelenlegi gyro érték * érzékenység

            calculatedAngle *= direction
            #* Ne forduljon meg a robot hátra menésnél

            if(abs(calculatedAngle) > 100): calculatedAngle = sign(calculatedAngle) * 100
            #* Ne tudjon a maximumnál nagyobb értékkel fordulni

            """if(gs.angle == irany):
                pontos += 1
            osszesMeres += 1"""
            #* Pontosságot számolja

            s.on(calculatedAngle, calculatedSpeed)
            #* Elindítja a motort a kiszámolt sebességel és szögben.
            previousAngle = calculatedAngle
            timesBad += 1
            timesGood = 0
        else:
            if(timesBad + margin != timesGood):
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

def gotoXY(targetX, targetY, maxSpeed, minSpeed, sensitvity, margin = 4, speedUp =0.3, slowDown = 0.8, debug = False, motorStop = False, shouldSlow = True, curve = False):
    global currentX
    global currentY
    global rotations
    rotations = getRotations()

    distance = (abs(abs(targetX) - abs(currentX))) + abs((abs(targetY) - abs(currentY)))

    startDistance = distance

    speedUp *= startDistance
    slowDown *= startDistance





    if(curve == True):
        coordinates = findBigger((targetX - currentX), (targetY - currentY))
        curveX = coordinates[0] * sign((targetX - currentX))
        curveY = coordinates[1] * sign((targetY - currentY))
        curveTargetAngle = math.degrees(math.atan2(curveX, curveY))
        startCurveDistance = abs(abs(abs(abs(targetX) - abs(curveX)) - abs(currentX)) + abs(abs(abs(targetY) - abs(curveY)) - abs(currentY)))
    #! KISZÁMÍT IRÁNY AMIBE KEVESEBBET KELL MENNI, TARGET IRÁNY = MIUTÁN ELÉRTED A KISEBB IRÁNYBÓL A CÉL ÉRTÉKET MILYEN SZÖGBEN KELL MENN
    #! TARGET IRÁNY = AMEDDIG AZ EGYIK IRÁNY EL NEM ÉRI DECIMALBA ÁTVÁLTVA, SZOROZVA, 

    while abs(abs(targetX) - abs(currentX)) > margin or abs(abs(targetY) - abs(currentY)) > margin:
        status = "Starting"

        

        distance = (abs(abs(targetX) - abs(currentX))) + abs((abs(targetY) - abs(currentY)))

        currentX += (math.sin(math.radians(gs.angle)) * ((getRotations() - rotations))) * -1
        currentY += math.cos(math.radians(gs.angle)) * ((getRotations() - rotations))
        rotations = getRotations()

        if(curve == True):
            distanceDecimal = 1- (abs(abs(abs(abs(targetX) - abs(curveX)) - abs(currentX)) + abs(abs(abs(targetY) - abs(curveY)) - abs(currentY))) / startCurveDistance)
            targetAngle = curveTargetAngle * distanceDecimal
        else:
            targetAngle = math.degrees(math.atan2(targetX - currentX, targetY - currentY))

        if (abs((gs.angle * -1) - targetAngle) > 180):
            targetAngle = 360 - abs(targetAngle)

        calculatedSpeed = calculateSpeed(distance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, startDistance)

        #* Ha nem gyorsul vagy lassul akkor maximum sebességel menjen
        if(abs(calculatedSpeed) > abs(maxSpeed)): calculatedSpeed = (abs(calculatedSpeed) / calculatedSpeed) * abs(maxSpeed)
        #* Ne tudjon véletlenül sem a maximum sebességnél gyorsabban menni

        calculatedAngle = ((gs.angle) + targetAngle) * sensitvity

        print("Target: " + str(targetAngle) + "\tCalculated: " + str(calculatedAngle) + "\tCurrent: " + str(gs.angle))

        if(debug == True):
            print("----------------------------")
            print("X: "+str(round(currentX, 2)) + "\tY: " + str(round(currentY, 2)))
            print("targetX: "+str(targetX) + "\ttargetY: " + str(targetY))
            print("Current Angle: " + str(gs.angle) + "\tTarget Angle: " + str(round(targetAngle, 2)))
            print("Left speed: " + str(leftm.speed) + "\tRight speed: " + str(rightm.speed))
            #print("slowDwon: "+str(slowDown) + "\tspeedUp: "+str(speedUp))
            #print("Predicted Distance: " + str(distance) + "\tStart Distance: " + str(startDistance))
            #print("Status: " + status)
        
        
        #~     gyro célérték     jelenlegi gyro érték * érzékenység

        if(maxSpeed < 0):
            calculatedAngle = calculatedAngle
        #* Ne forduljon meg a robot hátra menésnél

        if(abs(calculatedAngle) > 100): calculatedAngle = (abs(calculatedAngle) / calculatedAngle) * 100
        s.on(calculatedAngle, calculatedSpeed)
        time.sleep(0.01)
    m.stop()

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

    time.sleep(0.1)
    straight(float(dist) + 2, -50, 0, 1.1, 5, False, False, False, True, True, 0, 0)



    print(wheelDiameter)
rotations = getRotations()
input("Start? ")

currentX = 40
currentY = 40
gs.reset()
leftm.reset()
rightm.reset()
try:
    gotoXY(170, 80, 50.00, 5.00, 0.8, 4, 0.30, 0.7, curve=True)
    gotoXY(170, 80, 50.00, 5.00, 0.8, 4, 0.30, 0.7, curve=True)
    gotoXY(170, 35, 50.00, 5.00, 0.8, 4, 0.30, 0.7, curve=True)
    gotoXY(37, 20.5, 50.00, 5.00, 0.8, 2, 0.30, 0.7, curve=True)
    #gotoXY(40, 40, 20, 5.00, 0.8, 1.5, 0.30, 0.7, True)

except KeyboardInterrupt:
    m.stop()
    m.reset()
    print("Exited the program")
m.stop()
m.reset()
    
