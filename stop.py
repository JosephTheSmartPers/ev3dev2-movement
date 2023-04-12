from ev3dev2.motor import OUTPUT_B, OUTPUT_C, OUTPUT_D, OUTPUT_A, MoveTank, MediumMotor
print("ready")
m = MoveTank(OUTPUT_B, OUTPUT_C)
MediumMotor(OUTPUT_D).reset()
MediumMotor(OUTPUT_D).stop()
MediumMotor(OUTPUT_A).reset()
MediumMotor(OUTPUT_A).stop()
m.stop()
m.reset()