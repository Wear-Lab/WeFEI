import pyrealsense2 as rs
import numpy as np
import cv2
import os
import math
from rotations import bodyToInertialFrame, inertialToBodyFrame

MODEL = "yolov4-tiny"
DATA = "coco"
FRAME_SCALE_FACTOR = 1.5
# Temp until we have command recognition
TARGET_OBJECT = "mouse"

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    conf.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
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

            rotationAngles[x] = accelAngle[x]
            rotationAngles[y] = accelAngle[y]
            rotationAngles[z] = accelAngle[z]

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

        counter += 1
        if counter > 50:
            print("rotationAngles (pitch, yaw, roll): (" + str(math.degrees(rotationAngles[x])) + ", " + str(math.degrees(rotationAngles[y])) + ", " + str(math.degrees(rotationAngles[z])) + ")")
            bodyAccel = inertialToBodyFrame(rotationAngles, accel)
            print("Body Accel (X, Y, Z): (" + str(bodyAccel) + ")")
            counter = 0

finally:
    p.stop()
