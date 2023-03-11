#!/usr/bin/env python3
from ev3dev2.display import Display
from time import sleep

lcd = Display()

# Draw a circle of ‘radius’ centered at (x, y)
lcd.circle(clear_screen=False, x=89, y=64, radius=61, fill_color='lightgrey')
lcd.circle(False, x=65, y=45, radius=10, fill_color='black')
lcd.circle(False, x=113, y=45, radius=10, fill_color='black')

# Draw a line from (x1, y1) to (x2, y2)
lcd.line(False, x1=89, y1=50, x2=89, y2=80, line_color='grey', width=4)

# Draw a rectangle where the top left corner is at (x1, y1)
# and the bottom right corner is at (x2, y2). 
lcd.rectangle(False, x1=69, y1=90, x2=109, y2=105, fill_color='grey')

# Draw a single pixel at (x, y)
lcd.point(False, x=65, y=45, point_color='white')
lcd.point(False, x=113, y=45, point_color='white')

lcd.update()
sleep(10)