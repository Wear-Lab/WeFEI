import pyrealsense2 as rs
from time import time
import numpy as np
import pyyolo
import cv2
import os

FPS_COUNTER_MAX = 200
MODEL = "yolov4-tiny"
DATA = "coco"
FRAME_SCALE_FACTOR = 1.5
# Temp until we have command recognition
TARGET_OBJECT = "clock"

def main():

	pipeline = rs.pipeline()
	config = rs.config()

	config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
	pipeline.start(config)

	detector = pyyolo.YOLO("./models/" + MODEL  + ".cfg",
                           "./models/" + MODEL + ".weights",
                           "./models/" + DATA + ".data",
                           detection_threshold = 0.5,
                           hier_threshold = 0.5,
                           nms_threshold = 0.45)

	cap = cv2.VideoCapture(2)

	fpsFile = open("fpsMeasurements.txt", "a")
	fpsMeasurements = [];
	fpsMeasureCounter = 0
	oldTime = int(time() * 1000)

	while True:
        	# Get RealSense frame first so we can guarantee we have one
		frames = pipeline.wait_for_frames()

		# Frames 1280 width x 720 height
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()
		if not depth_frame or not color_frame:
			continue
        
		# Check PyYOLO frame for target object
		# This frame is 1920 width x 1080 height
		ret, frame = cap.read()
		dets = detector.detect(frame, rgb=False)

		if fpsMeasureCounter < FPS_COUNTER_MAX:
			newTime = int(time() * 1000)
			diff = newTime - oldTime
			fps = 1000 / diff
			fpsMeasurements.append(fps);
		elif fpsMeasureCounter == FPS_COUNTER_MAX:
			fpsSum = 0
		for val in fpsMeasurements:
			fpsSum = fpsSum + val
			fpsFile.write("Average FPS with " + MODEL + " = " + str(fpsSum / FPS_COUNTER_MAX) + " FPS\n")

		# To avoid eventual integer overflow, stop counting frames
		if fpsMeasureCounter <= FPS_COUNTER_MAX: 
			fpsMeasureCounter = fpsMeasureCounter + 1

		oldTime = newTime

		for i, det in enumerate(dets):
			# Don't print frames if we've stopped counting
			if fpsMeasureCounter > FPS_COUNTER_MAX:
				print("[Past Frame Count Limit] Detection: " + str(i) + ", " + str(det))
			# Count frames as it approaches FPS_COUNTER_MAX
			else:
				print("Frame " + str(fpsMeasureCounter) + " - Detection: " + str(i) + ", " + str(det))
			xmin, ymin, xmax, ymax = det.to_xyxy()
			cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255))
			cv2.putText(frame, det.name + "," + str(det.prob), (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (107, 168, 50), 1);

		cv2.imshow("Preview Window", frame)

		if cv2.waitKey(1) == 27:
			break

if __name__ == '__main__':
    main()
