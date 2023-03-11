from ev3dev2.motor import OUTPUT_B, OUTPUT_C, MoveTank
print("ready")
m = MoveTank(OUTPUT_B, OUTPUT_C)
m.stop()
m.reset()