class Dexter:
  def __init__(self, motorLeft, motorRight, handX, handY, gyro, colorSensorLeft, colorSensorRight, diagnostics = False):
    ports = [motorLeft, motorRight, handX, handY, gyro, colorSensorLeft, colorSensorRight]
    portNames = ["motorLeft", "motorRight", "handX", "handY", "gyro", "colorSensorLeft", "colorSensorRight"]

    self.motorLeft, self.motorRight, self.handX, self.handY, self.gyro, self.colorSensorLeft, self.colorSensorRight = motorLeft, motorRight, handX, handY, gyro, colorSensorLeft, colorSensorRight

    
    

p1 = Dexter("outB", "outC", "outD", "outA", "in2", "in4", "in3")

