from ev3dev2.motor import LargeMotor, MoveTank, MediumMotor
#? Imports
MediumMotor("A").on_for_rotations(-100, 2)
MediumMotor("A").on_for_rotations(100, 2)