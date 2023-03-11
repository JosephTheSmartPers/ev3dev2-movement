from math import atan2, degrees

curveX = 20
curveY = 0
currentX = 200
currentY = 300

targetX = 50
targetY = 30

startCurveDistance = (abs(abs(abs(abs(targetX) - abs(curveX)) - abs(currentX + 100)) + abs(abs(abs(targetY) - abs(curveY)) - abs(currentY + 100))))


decimal = 1- (abs(abs(abs(abs(targetX) - abs(curveX)) - abs(currentX)) + abs(abs(abs(targetY) - abs(curveY)) - abs(currentY))) / startCurveDistance)
print(decimal)

print(degrees(atan2(2, 30)))