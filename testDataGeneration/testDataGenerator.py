import pyrealsense2 as rs
import numpy as np
import math
import time

def initializeCamera():
	# start the frames pipe
	p = rs.pipeline()
	conf = rs.config()

	conf.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 250)
	conf.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
	prof = p.start(conf)
	return p


def main():
	p = initializeCamera()
	frameCounter = 0
	initialTS = time.time()
	prevTS = time.time()
	fileSuffix = 0
	fp = None

	while(True):
		try:
			fp = open(f'testData{fileSuffix}.txt', "x")
			break
		except:
			print(f'testData{fileSuffix}.txt taken')
			fileSuffix+=1

	fp.write('Acc_X,Acc_Y,Acc_Z,Gyr_X,Gyr_Y,Gyr_Z\n')

	try:
		while prevTS - initialTS <= 5:
			now = time.time()
			frameSet = p.wait_for_frames()
			fp.write(f' ')
			for i in range(0, frameSet.size()):
				if frameSet[i].is_motion_frame():
					frame = frameSet[i].as_motion_frame()
					if frame.get_profile().stream_type() == rs.stream.accel:
						accel = frame.get_motion_data() # accel.x/y/z
						fp.write(f"{accel.x:.6f},{accel.y:.6f},{accel.z:.6f},")
					elif frame.get_profile().stream_type() == rs.stream.gyro:
						gyro = frame.get_motion_data() # gyro.x/y/z
						fp.write(f"{gyro.x:.6f},{gyro.y:.6f},{gyro.z:.6f}\n")
			
			frameCounter += 1
			prevTS = now
	finally:
		p.stop()
		fp.close()
		print("Avg Rate = ", frameCounter / (time.time() - initialTS))


main()