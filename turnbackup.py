def fordul(szog, maxSebesseg, sensitivity, koIdo, hibahatar = 2, idotullepes = 2, relativ = True, motorLe = False, waitMargin = 2, stopAfter = 1, debug = False, minSpeed = 2):
    """Makes the motor turn in a specified degree"""
    kezdoIdo = time()
    #? Fordulás kezdetének időpontja
    elozoIdo = 999999999999
    elteltIdo = time() - elozoIdo
    timesCorrect = 0
    hasStopped = False
    hasRecalculated = False
    #? Now nagy szám, mivel kivonjuk a mostani időből, mivel ezt nézi az egyik programrész
    szog = szog * -1
    #? Így megy a jó irányba, gyro meg van fordítva
    fordulatszam = 0
    if(relativ == True):
        fordulatszam = gs.angle
    prevTime = time()
    deltaTime = 0.1
    prevDegree = gs.angle
    deltaDegree = 1
    prevSpeed = 1
    #* 0-hoz képest vagy a gyrohoz képest forduljon el adott szögben
    while gs.angle != fordulatszam - szog or timesCorrect <= waitMargin :
        if(gs.angle == fordulatszam - szog):
            timesCorrect += 1
        elif(timesCorrect != 0):
            timesCorrect = 0 
        deltaTime = optimizeFloat(time() - prevTime)
        prevTime = time()
        estimatedTurn = optimizeFloat(deltaTime * deltaDegree * 35)
        estimatedTurn += sign(estimatedTurn) 
        #? Using Runge Kuta 4 to aproximate the amount the robot will turn in the next cycle and stopping the motors if it might turn too much
        if((fordulatszam-szog) - estimatedTurn <= gs.angle <= (fordulatszam - szog) + estimatedTurn and hasStopped == False and stopAfter != 0):
            hasStopped = True
            print("what am i doing?")
            m.stop()
            continue
        if((fordulatszam-szog) - stopAfter <= gs.angle <= (fordulatszam - szog) + stopAfter and hasStopped == False and stopAfter != 0):
            hasStopped = True
            print("e")
            m.stop()
            continue
        if(kezdoIdo + idotullepes <= time()):
            break
        #* Ha túl sokáig csinálja a fordulást akkor abbahagyja, mert lehet, hogy be van akadva
        elteltIdo = time() - elozoIdo
        #* Ez egy nagy számmal negatív lesz, ha viszont az előzőidő valóban az előző idő akkor megkapod 
        #* a két számolás közti különbséget, és ezt hozzáadjuk a változóhoz
        if(((fordulatszam - szog) - hibahatar <= gs.angle <= (fordulatszam - szog)  + hibahatar) and elozoIdo > time()):
            elozoIdo = time()
        #* Ha már közel van a giroszkóp a célértékhez, akkor elkezdi mérni az időt
        if(elteltIdo >= koIdo):
            m.stop()
            break
        #* Ha a robot már közel van a cél szöghöz, akkor lesz még egy adott ideje, hogy kisebbet korigáljon, aztán abbahagyja
        #linearPlease = (fordulatszam - szog) * 1.5
        #calculatedSpeed = abs(linearPlease - gs.angle) * sensitivity
        calculatedSpeed = ((fordulatszam - szog) - gs.angle) * sensitivity
        if(calculatedSpeed > maxSebesseg / 4):
            calculatedSpeed = maxSebesseg
        deltaDegree = prevDegree - gs.angle
        prevDegree = gs.angle
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
    print("\nDONE\n")
    #print("Kész a fordulás, célértéktől való eltérés: " + str(round(float(abs((((gs.angle / szog) * 100) - 100))), 2)) + "%")