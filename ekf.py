from ev3dev2.motor import LargeMotor, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import GyroSensor
from math import pi
import numpy as np
import time

# Initialize the gyroscope and motors
gyro = GyroSensor()
motor_left = LargeMotor(OUTPUT_B)
motor_right = LargeMotor(OUTPUT_C)

# Define the motion model
def motion_model(x, u):
    dt = 0.1
    theta = x[0]
    w = u[0]
    theta_new = theta + w * dt
    return np.array([theta_new])

# Define the sensor model
def sensor_model(x):
    return np.array([x[0]])

# Initialize the state estimate and covariance matrix
x = np.array([gyro.angle])
P = np.array([[0.1]])

# Specify the target degree to turn
target_degree = 90

# Run the EKF for turning
while abs(gyro.angle - target_degree) > 1:
    # Predict the state
    w = (motor_right.position - motor_left.position) / (2 * pi * 0.055 * 0.5)
    u = np.array([w])
    x = motion_model(x, u)
    F = np.array([[1]])
    Q = np.array([[0.001]])
    P = (np.eye(1) - K @ H) @ P

    # Update the state estimate
    z = np.array([gyro.angle])
    H = np.array([[1]])
    R = np.array([[0.01]])
    y = z - sensor_model(x)
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x + K @ y
    P = (np.eye(1) - K @ H) @ P

    # Control the motors
    if abs(gyro.angle - target_degree) > 10:
        motor_left.run_forever(speed_sp=-200)
        motor_right.run_forever(speed_sp=200)
    else:
        motor_left.run_forever(speed_sp=-100)
        motor_right.run_forever(speed_sp=100)

    # Wait for some time
    time.sleep(0.1)

# Stop the motors
motor_left.stop()
motor_right.stop()
