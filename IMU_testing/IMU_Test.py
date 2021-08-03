import pyrealsense2 as rs
import numpy as np
import math
import time
from rotations import bodyToInertialFrame, inertialToBodyFrame

def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()

    conf.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
    conf.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    conf.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    conf.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    prof = p.start(conf)
    return p

def main():
    p = initialize_camera()

    first = True
    alpha = 0.98
    last_ts_gyro = time.time() * 1000
    counter = 0

    x = 0
    pitch = 0

    y = 1
    yaw = 1

    z = 2
    roll = 2

    rotationAngles = [0.0] * 3  # [Pitch, Yaw, Roll] or [x, y, z]
    accelAngle = [0.0] * 3
    gyroAngle = [0.0] * 3

    cameraPos = [0.0] * 3
    cameraVel = [0.0] * 3

    try:
        while True:
            f = p.wait_for_frames()

            for i in range(0, f.size()):
                if f[i].is_motion_frame():
                    frame = f[i].as_motion_frame()
    
                    if frame.get_profile().stream_type() == rs.stream.accel:
                        accel = frame.get_motion_data()

                        print("Accel (body): [" + str(round(accel.x, 5)) + ", " + str(round(accel.y, 5)) + ", " + str(round(accel.z, 5)) + "]")

                        # Use acceleration data
                        accelAngle[x] = math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z))
                        accelAngle[z] = math.atan2(accel.y, accel.z)
    
                        if (first):
                            first = False
                            accelAngle[y] = math.pi
                            rotationAngles = accelAngle
                        else:
                            rotationAngles[x] = rotationAngles[x] * alpha + accelAngle[x] * (1 - alpha)
                            rotationAngles[z] = rotationAngles[z] * alpha + accelAngle[z] * (1 - alpha)

                    elif frame.get_profile().stream_type() == rs.stream.gyro:
                        gyro = frame.get_motion_data()
                        ts = frame.get_timestamp()
                        if first:
                            last_ts_gyro = ts
                            return

                        # Gyro calculations
                        dt_gyro = (ts - last_ts_gyro) / 1000
                        last_ts_gyro = ts
                    
                        gyroAngle[x] = gyro.x * dt_gyro
                        gyroAngle[y] = gyro.y * dt_gyro
                        gyroAngle[z] = gyro.z * dt_gyro

                        rotationAngles[x] -= gyroAngle[z]
                        rotationAngles[y] -= gyroAngle[y]
                        rotationAngles[z] += gyroAngle[x]

                # Would handle depth and color stuff from here (non-motion-frames)

            print("Rotation Angles (r, y, p): [" + str(round(rotationAngles[roll] * 180 / math.pi, 5)) + ", " + str(round(rotationAngles[yaw] * 180 / math.pi, 5)) + ", " + str(round(rotationAngles[pitch] * 180 / math.pi, 5)) + "]")

            newVec = inertialToBodyFrame(rotationAngles, [0.0, 9.81, 0.0])

            print("Gravity in body frame: [" + str(round(newVec[x], 5)) + ", " + str(round(newVec[y], 5)) + ", " + str(round(newVec[z], 5)) + "] w/ len = " + str(round(math.sqrt(newVec[x] * newVec[x] + newVec[y] * newVec[y] + newVec[z] * newVec[z]), 5)))

            '''counter += 1
            if counter > 50:
                print("rotationAngles (pitch, yaw, roll): (" + str(round(math.degrees(rotationAngles[x]), 4)) + ", " + str(round(math.degrees(rotationAngles[y]), 4)) + ", " + str(round(math.degrees(rotationAngles[z]), 4)) + ")")
                print("Body Accel (X, Y, Z): (" + str(round(accel.x, 4)) + ", " + str(round(accel.y, 4)) + ", " + str(round(accel.z, 4)) + ")")
                inertialAccel = bodyToInertialFrame(rotationAngles, accel)
                print("Inertial Accel (X, Y, Z): (" + str(round(inertialAccel[x], 4)) + ", " + str(round(inertialAccel[y], 4)) + ", " + str(round(inertialAccel[z], 4)) + ")")

                # Apply kinematics to test camera location tracking
                # vf = vi + a t
                # d= vi t + 1/2 a t^2
                cameraPos[x] += cameraVel[x] * dt_gyro + .5 * inertialAccel[x] * dt_gyro * dt_gyro
                cameraPos[y] += cameraVel[y] * dt_gyro + .5 * inertialAccel[y] * dt_gyro * dt_gyro
                cameraPos[z] += cameraVel[z] * dt_gyro + .5 * inertialAccel[z] * dt_gyro * dt_gyro

                cameraVel[x] = cameraVel[x] + inertialAccel[x] * dt_gyro
                cameraVel[y] = cameraVel[y] + inertialAccel[y] * dt_gyro
                cameraVel[z] = cameraVel[z] + inertialAccel[z] * dt_gyro

                print("Camera Velocity (X, Y, Z): (" + str(round(cameraVel[x], 8)) + ", " + str(round(cameraVel[y], 8)) + ", " + str(round(cameraVel[z], 8)) + ")")
                print("Camera Position (X, Y, Z): (" + str(round(cameraPos[x], 8)) + ", " + str(round(cameraPos[y], 8)) + ", " + str(round(cameraPos[z], 8)) + ")")
                counter = 0'''

    finally:
        p.stop()

main()
