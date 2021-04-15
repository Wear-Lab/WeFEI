import pyrealsense2 as rs
import numpy as np
import math
from rotations import bodyToInertialFrame, inertialToBodyFrame

'''
This works!
Pitch goes -90 -> 0 -> 90 -> 0 degrees
Yaw goes infinitely in both directions, so I'm moding by 360 to keep it simple
Roll goes -180 -> 180 then immediately resets to -180, and the same for the other direction
'''

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p

p = initialize_camera()

first = True
alpha = 0.98
x = 0
y = 1
z = 2
counter = 0

rotationAngles = [0.0] * 3  # [Pitch, Yaw, Roll]
accelAngle = [0.0] * 3
gyroAngle = [0.0] * 3

try:
    while True:
        f = p.wait_for_frames()

        #gather IMU data
        accel = f[0].as_motion_frame().get_motion_data()
        gyro = f[1].as_motion_frame().get_motion_data()

        ts = f.get_timestamp()
        # accelerometer calculation
        accelAngle[x] = math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z))
        accelAngle[z] = math.atan2(accel.y, accel.z)

        #calculation for the first frame
        if (first):
            first = False
            last_ts_gyro = ts

            accelAngle[y] = math.pi

            rotationAngles[x] = accelAngle[x];
            rotationAngles[y] = accelAngle[y];
            rotationAngles[z] = accelAngle[z];

            continue

        #calculation for the second frame onwards

        # gyrometer calculations
        dt_gyro = (ts - last_ts_gyro) / 1000
        last_ts_gyro = ts

        gyroAngle[x] = gyro.x * dt_gyro
        gyroAngle[y] = gyro.y * dt_gyro
        gyroAngle[z] = gyro.z * dt_gyro

        rotationAngles[x] += -1 * gyroAngle[z]
        rotationAngles[y] += -1 * gyroAngle[y]
        rotationAngles[z] += gyroAngle[x]
	
        #combining gyrometer and accelerometer angles
        rotationAngles[x] = rotationAngles[x] * alpha + accelAngle[x] * (1-alpha)
        rotationAngles[z] = rotationAngles[z] * alpha + accelAngle[z] * (1-alpha)

        rotationAngles[x] = rotationAngles[x] % 360
        rotationAngles[y] = rotationAngles[y] % 360
        rotationAngles[z] = rotationAngles[z] % 360

        counter += 1
        if counter > 50:
            print("rotationAngles (pitch, yaw, roll): (" + str(math.degrees(rotationAngles[x])) + ", " + str(math.degrees(rotationAngles[y])) + ", " + str(math.degrees(rotationAngles[z])) + ")")
            print("Body Accel (X, Y, Z): (" + str(accel) + ")")
            inertialAccel = bodyToInertialFrame(rotationAngles, accel)
            print("Inertial Accel (X, Y, Z): (" + str(inertialAccel) + ")")
            counter = 0

finally:
    p.stop()
