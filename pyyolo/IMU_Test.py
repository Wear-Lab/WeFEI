import pyrealsense2 as rs
import numpy as np
import time
import math

theta = [0.0] * 3
alpha = 0.98
firstGyro = True
firstAccel = True
lastGyroTimestamp = 0

# FIX THETA, READ NOTES PAGE

def initialize_camera():
	# start the frames pipe
	p = rs.pipeline()
	conf = rs.config()
	conf.enable_stream(rs.stream.accel)
	conf.enable_stream(rs.stream.gyro)
	prof = p.start(conf)
	return p

def gyroToArray(gyro):
	return np.asarray([gyro.x, gyro.y, gyro.z])

def processGyro(gyro, ts):
	global theta, alpha, firstGyro, lastGyroTimestamp
	
	if firstGyro:
		firstGyro = False
		lastGyroTimestamp = ts
		return
	gyroAngle = [0.0] * 3
	gyroAngle[0] = gyro.x
	gyroAngle[1] = gyro.y
	gyroAngle[2] = gyro.z

	tsDiff = (ts - lastGyroTimestamp) / 1000.0
	lastGyroTimestamp = ts

	gyroAngle[0] *= tsDiff
	gyroAngle[1] *= tsDiff
	gyroAngle[2] *= tsDiff

	theta[0] += -1 * gyroAngle[2]
	theta[1] += -1 * gyroAngle[1]
	theta[2] += gyroAngle[0]

def accelToArray(accel):
	return np.asarray([accel.x, accel.y, accel.z])

def processAccel(accel):
	global theta, alpha, firstAccel, lastGyroTimestamp

	accelAngle = [0.0] * 3

	accelAngle[2] = math.atan2(accel.y, accel.z)
	accelAngle[0] = math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z)) 

	if firstAccel:
		firstAccel = False
		theta = accelAngle
		theta[1] = math.pi
	else:
		theta[0] = theta[0] * alpha + accelAngle[0] * (1 - alpha)
		theta[2] = theta[2] * alpha + accelAngle[2] * (1 - alpha)

# MAIN
p = initialize_camera()
try:
	while True:
		f = p.wait_for_frames()
		accel = f[0].as_motion_frame().get_motion_data()
		gyro = f[1].as_motion_frame().get_motion_data()
		
		processAccel(accel)
		processGyro(gyro, time.time())

		print("accelerometer: ", accelToArray(accel))
		print("gyro: ", gyroToArray(gyro))
		print("theta: " + str(theta))
		time.sleep(0.5)
finally:
	p.stop()


# THIS AIN'T WORKING

