from math import sin, cos, degrees, atan2, radians, sqrt

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

controlPoints = [Vector(75, 25), Vector(25, 25), Vector(0,0)]


previousPoint = getPointOnBezier(controlPoints, 0)
print(bezierLenght(controlPoints))
"""for i in range(0, 100):
    currentPoint = getPointOnBezier(controlPoints,(i/100))
    print(degrees(atan2(currentPoint.y - previousPoint.y, currentPoint.x - previousPoint.x)))
    previousPoint = currentPoint"""
    