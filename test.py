from ev3dev2.motor import MoveTank, LargeMotor, MoveSteering
from ev3dev2.sensor.lego import GyroSensor, ColorSensor
import time
#? Mindent beimportálunk

wheelDiameter = 1
leftm = LargeMotor("outB")
rightm = LargeMotor("outC")
m = MoveTank("outB", "outC")
s = MoveSteering("outB", "outC")
gs = GyroSensor("in2")
gs.reset()
rightSensor = ColorSensor("in3")
leftSensor = ColorSensor("in4")
def getRotations():
    return (leftm.rotations + rightm.rotations) / 2 * wheelDiameter
def sign(num):
    if(num == 0):
        return 1
    return(num / abs(num))

def calculateSpeed(currentDistance, startDistance, speedUp, slowDown, maxSpeed, minSpeed, motorStop, shouldSlow, distance):
    """This function calculates the speed, for linearly speeding up and slowing down."""

    deltaDistance = abs(abs(currentDistance) - startDistance)


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
def fordul(szog, maxSebesseg, sensitivity, koIdo, hibahatar = 2, idotullepes = 2, relativ = True, motorLe = False, waitMargin = 2, stopAfter = 1, debug = False, minSpeed = 2):
    """Makes the motor turn in a specified degree"""

    kezdoIdo = time.time()
    #? Fordulás kezdetének időpontja

    elozoIdo = 999999999999
    elteltIdo = time.time() - elozoIdo
    timesCorrect = 0

    hasStopped = False
    hasRecalculated = False
    #? Now nagy szám, mivel kivonjuk a mostani időből, mivel ezt nézi az egyik programrész

    szog = szog * -1
    #? Így megy a jó irányba, gyro meg van fordítva

    fordulatszam = 0
    if(relativ == True):
        fordulatszam = gs.angle

    prevTime = time.time()
    deltaTime = 0.1
    prevDegree = gs.angle
    deltaSpeed = 0
    prevSpeed = maxSebesseg
    deltaDegree = 1
    esitmatedSum = 0
    #* 0-hoz képest vagy a gyrohoz képest forduljon el adott szögben
    
    while gs.angle != fordulatszam - szog or timesCorrect <= waitMargin :

        if(gs.angle >= 90):
            m.stop()
            break

        calculatedSpeed = ((fordulatszam - szog) - gs.angle) * sensitivity
        deltaDegree = prevDegree - gs.angle
        prevDegree = gs.angle
        deltaSpeed = prevSpeed - calculatedSpeed
        prevSpeed = calculatedSpeed
        #* Kiszámolja a sebességet, ami egyre lassul minnél közelebb vagy a cél giroszkóp értékhez
        m.on(-calculatedSpeed, calculatedSpeed)


    if(motorLe != False):
        m.on(motorLe, motorLe)
    else:
        m.stop()
    
    
    


    #print("Kész a fordulás, célértéktől való eltérés: " + str(round(float(abs((((gs.angle / szog) * 100) - 100))), 2)) + "%")

drift = -2
gs.reset()

s.on_for_rotations(-2, 40, 1)


fordul(90, 45, 0.48, 0.5, idotullepes=3, relativ=False, stopAfter=3, waitMargin=1, debug=False, minSpeed=4)
print(gs.angle)
time.sleep(1)
print(gs.angle)
s.on_for_rotations(-2, 40, 1)
