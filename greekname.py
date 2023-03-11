import random as r
import tkinter

suernames = ["goras", "ates", "ades", "totle", "genes"]
after = r.choices(suernames)
print(after)
nev = ""
vege = ["a", "o"]
magan = ["a", "e", "i", "o", "u"]
masal = ["b", "c", "d", "f", "g", "h", "k", "l", "m", "n", "p", "r", "s", "t", "v", ]
after = str(after).replace("['", "")
after = after.replace("']", "")
if(r.uniform(0, 1) == 0):
    nev = str(r.choice(magan)).upper() + str(r.choice(masal)) + str(r.choice(magan)) + str(after)
else:
    nev = str(r.choice(masal)).upper() + str(r.choice(magan)) + str(r.choice(masal)) + str(r.choice(vege)) + str(after)
print(nev)