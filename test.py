import math

tan = math.tan(70)
xcos = math.cos(tan)
xlen = 5000
#5000 = x * xcos

clen = xlen / xcos
print(clen)

ylen = math.sqrt(clen**2 - xlen**2)

confirmation = math.sqrt(ylen**2 + xlen**2)

diameter = confirmation * math.pi
#diameter stadionokban

stadionlen = 155
#155 m

print(diameter * stadionlen)