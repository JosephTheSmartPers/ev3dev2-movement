from ev3dev2.motor import LargeMotor, OUTPUT_C, OUTPUT_B, SpeedPercent, MoveTank, MoveSteering
s = MoveSteering(OUTPUT_C, OUTPUT_B)
m = MoveTank(OUTPUT_C, OUTPUT_B)
m.on(75, 75)