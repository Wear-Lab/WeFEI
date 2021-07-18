import pyrealsense2 as rs
import numpy as np
import pyyolo
import cv2
import os
from rotations import bodyToInertialFrame, inertialToBodyFrame

MODEL = "yolov4-tiny"
DATA = "coco"
FRAME_SCALE_FACTOR = 1.5
# Temp until we have command recognition
TARGET_OBJECT = "mouse"

def main():

	pipeline = rs.pipeline()
	config = rs.config()

	config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
	pipeline.start(config)

	profile = pipeline.get_active_profile()
	depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
	intr = depth_profile.get_intrinsics()

	detector = pyyolo.YOLO("./models/" + MODEL  + ".cfg",
		"./models/" + MODEL + ".weights",
		"./models/" + DATA + ".data",
		detection_threshold = 0.5,
		hier_threshold = 0.5,
		nms_threshold = 0.45)

	while True:
		# Get RealSense frame first so we can guarantee we have one
		frames = pipeline.wait_for_frames()

		# Frames 1280 width x 720 height
		#get_distance(x: int, y: int) -> float
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()

		if not depth_frame or not color_frame:
			continue
		
		# Convert images to numpy arrays
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		dets = detector.detect(color_image, rgb=False)

		for i, det in enumerate(dets):

			if det.name != TARGET_OBJECT:
				continue

			'''
			TODOS here: Select target object based on how many frames it shows up in one second with high enough confidence
					From there, take the last frame it was found and calculate depth based on that frame.
			'''

			xmin, ymin, xmax, ymax = det.to_xyxy()
			cv2.rectangle(color_image, (xmin, ymin), (xmax, ymax), (0, 0, 255))
			cv2.putText(color_image, det.name + "," + str(det.prob), (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (107, 168, 50), 1);

			found = False
			xcenter = int((xmin + xmax)/2)
			ycenter = int((ymin + ymax)/2)
			checkNorth = checkSouth = ycenter
			checkEast = checkWest = xcenter
			float_distance = 0

			while checkNorth >= ymin and checkEast <= xmax and checkSouth <= ymax and checkWest >= xmin:
				float_distance = depth_frame.get_distance(xcenter, checkNorth)
				if float_distance != 0:
					found = True
					break
				else:
					checkNorth -= 10

				float_distance = depth_frame.get_distance(checkEast, ycenter)
				if float_distance != 0:
					found = True
					break
				else:
					checkEast += 10

				float_distance = depth_frame.get_distance(xcenter, checkSouth)
				if float_distance != 0:
					found = True
					break
				else:
					checkSouth += 10

				float_distance = depth_frame.get_distance(checkWest, ycenter)
				if float_distance != 0:
					found = True
					break
				else:
					checkWest -= 10

			cv2.circle(color_image, (xcenter, ycenter), 10, (87, 134, 255), 3)
			cv2.putText(color_image, (str(float_distance) + "m") if found else "Not Available", (xcenter - 20, ycenter - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (87, 134, 255), 1)
			point3D = rs.rs2_deproject_pixel_to_point(intr, [xcenter, ycenter], float_distance)
			#print("Body Frame: " + str(point3D))
			#print("Inertial Frame: " + str(bodyToInertialFrame(point3D)))

		cv2.imshow("color_image preview", color_image)
		if cv2.waitKey(1) == 27:
			break

	pipeline.stop()

if __name__ == '__main__':
	main()
