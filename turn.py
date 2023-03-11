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

def straight(distance, maxSpeed, targetAngle, sensitivity, minSpeed, stopOnLine = False, goOnLine = False, motorStop = False, shouldSlowDown = True, drift = 0, margin = 0, calibrate = False, debug = False):
    """Make the robot go staright in a specified degree (cm)"""

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
                s.on(-previousAngle, calculatedSpeed)
            else:
                s.on(drift, calculatedSpeed)
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
    #* Ha így bekapcsolva marad a 

def fordul(szog, maxSebesseg, sensitivity, koIdo, hibahatar = 2, idotullepes = 2, relativ = True, motorLe = False, waitMargin = 2, stopAfter = 1, debug = False, minSpeed = 2):
    """Makes the motor turn in a specified degree"""

    kezdoIdo = time.time()
    #? Fordulás kezdetének időpontja

    elozoIdo = 999999999999
    elteltIdo = time.time() - elozoIdo
    timesCorrect = 0

    hasStopped = False
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
    #* 0-hoz képest vagy a gyrohoz képest forduljon el adott szögben
    
    while gs.angle != fordulatszam - szog or timesCorrect <= waitMargin :

        if(gs.angle == fordulatszam - szog):
            timesCorrect += 1
        elif(timesCorrect != 0):
            timesCorrect = 0 

        deltaTime = prevTime - time.time()
        prevTime = time.time()

        k1 = deltaTime * deltaDegree
        k2 = deltaTime * (deltaDegree + 0.5 * k1)
        k3 = deltaTime * (deltaDegree + 0.5 * k2)
        k4 = deltaTime * (deltaDegree + k3)
        estimatedTurn = (k1 + 2 * k2 + 2 * k3 + k4) / 6 * 7.5
        #? Using Runge Kuta 4 to aproximate the amount the robot will turn in the next cycle and stopping the motors if it might turn too much
        
        if(sign((fordulatszam - szog) - gs.angle) != sign((fordulatszam - szog) - (gs.angle + round(float(estimatedTurn)))) and hasStopped == False):
            m.stop()
            hasStopped = True
            print("alert")
            continue

        if(gs.angle + stopAfter > (fordulatszam-szog) and (fordulatszam-szog) > gs.angle - stopAfter and hasStopped == False and stopAfter != 0):
            hasStopped = True
            m.stop()
            continue
            

        if(kezdoIdo + idotullepes <= time.time()):
            break
        #* Ha túl sokáig csinálja a fordulást akkor abbahagyja, mert lehet, hogy be van akadva

        elteltIdo = time.time() - elozoIdo
        #* Ez egy nagy számmal negatív lesz, ha viszont az előzőidő valóban az előző idő akkor megkapod 
        #* a két számolás közti különbséget, és ezt hozzáadjuk a változóhoz

        if(((fordulatszam - szog) - hibahatar <= gs.angle <= (fordulatszam - szog)  + hibahatar) and elozoIdo > time.time()):
            elozoIdo = time.time()
        #* Ha már közel van a giroszkóp a célértékhez, akkor elkezdi mérni az időt

        if(elteltIdo >= koIdo):
            m.stop()
            break
        #* Ha a robot már közel van a cél szöghöz, akkor lesz még egy adott ideje, hogy kisebbet korigáljon, aztán abbahagyja

        #linearPlease = (fordulatszam - szog) * 1.5
        #calculatedSpeed = abs(linearPlease - gs.angle) * sensitivity

        calculatedSpeed = ((fordulatszam - szog) - gs.angle) * sensitivity
        deltaDegree = prevDegree - gs.angle
        prevDegree = gs.angle
        deltaSpeed = prevSpeed - calculatedSpeed
        prevSpeed = calculatedSpeed
        #* Kiszámolja a sebességet, ami egyre lassul minnél közelebb vagy a cél giroszkóp értékhez


        if(abs(calculatedSpeed) > abs(maxSebesseg)): calculatedSpeed = (abs(maxSebesseg) * sign(calculatedSpeed))

        if(sign(fordulatszam - szog) != sign((fordulatszam - szog) -gs.angle)):
            if(abs(calculatedSpeed) < abs(minSpeed)): 
                calculatedSpeed = abs(minSpeed) * sign(calculatedSpeed)

        #* Ne lépje túl megadott felső sebességkorlátot 

        if(debug == True):
            print("Current Angle: " + str(gs.angle) + "\tTarget angle: " + str(fordulatszam-szog))

        m.on(-calculatedSpeed, calculatedSpeed)
        #* Elindítja a motorokat a forduláshoz ellenkező irányokba.


    if(motorLe != False):
        m.on(motorLe, motorLe)
    else:
        m.stop()
    
    


    #print("Kész a fordulás, célértéktől való eltérés: " + str(round(float(abs((((gs.angle / szog) * 100) - 100))), 2)) + "%")

"""drift = -2
gs.reset()

straight(1, 35, 0, 1.2, 5, False, False, False, True, True, -2, 0)



fordul(90, 45, 0.48, 0.5, idotullepes=3, relativ=False, stopAfter=3, waitMargin=1, debug=False, minSpeed=4)
print(gs.angle)
fordul(-90, 50, 0.58, 0.5, idotullepes=3, relativ=False, stopAfter=0, waitMargin=1, debug=False, minSpeed=1.5)
print(gs.angle)
fordul(1, 45, 0.48, 0.5, idotullepes=2, relativ=False, stopAfter=3, waitMargin=1, debug=False, minSpeed=3)
print(gs.angle)

straight(1, -35, 0, 1.2, 5, False, False, False, True, True, -2, 0)"""