import pyrealsense2 as rs
import numpy as np
import pyyolo
import cv2
import os

print(np.__version__)
print(pyyolo.__version__)
print(cv2.__version__)

MODEL = "yolov4-tiny"
DATA = "coco"
FRAME_SCALE_FACTOR = 1.5
# Temp until we have command recognition
TARGET_OBJECT = "mouse"

def main():

	# Set up RealSense camera to run depth and color streams (IMU streams not enabled yet)
	pipeline = rs.pipeline()
	config = rs.config()

	config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
	pipeline.start(config)

	profile = pipeline.get_active_profile()
	depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
	intr = depth_profile.get_intrinsics()

	# Setup pyyolo to run object recognition
	detector = pyyolo.YOLO("./models/" + MODEL  + ".cfg",
		"./models/" + MODEL + ".weights",
		"./models/" + DATA + ".data",
		detection_threshold = 0.5,
		hier_threshold = 0.5,
		nms_threshold = 0.45)

	while True:
		# Get RealSense frame first so we can guarantee we have one
		# The result is an array of two frames for one instance in time, one depth and one color
		frames = pipeline.wait_for_frames()

		# Frames 1280 width x 720 height
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()

		# As far as I can tell this never happens, but just safety
		if not depth_frame or not color_frame:
			continue
		
		# Convert images to numpy arrays
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		# Run detection on color stream (plain camera input)
		dets = detector.detect(color_image, rgb=False)

		# For each object the system picks up in the frame,
		for i, det in enumerate(dets):

			# Ignore non-target objects
			if det.name != TARGET_OBJECT:
				continue

			'''
			TODOS here: 
			- Select target object based on how many frames it shows up in one second with high enough confidence
			  From there, take the last frame it was found and calculate depth based on that frame.
			   		- Goal: Some detections are based on a singular frame and are gone the next, 
					   		or some are very wrong before correcting as the object shows more in frame
							We want to ignore any false positives and include only 'definite' findings
			'''

			# Show the object in the image when it renders to the screen
			xmin, ymin, xmax, ymax = det.to_xyxy()
			cv2.rectangle(color_image, (xmin, ymin), (xmax, ymax), (0, 0, 255))
			cv2.putText(color_image, det.name + "," + str(det.prob), (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (107, 168, 50), 1);

			# Starting from the center of the image, move outward until we have a concrete depth for the object
			# Not every point in the depth image has a reading every frame due to variation, 
			# so we don't want to take a 0 reading by accident

			# This might be overkill though
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

			# Render depth information for that center point
			cv2.circle(color_image, (xcenter, ycenter), 10, (87, 134, 255), 3)
			cv2.putText(color_image, (str(float_distance) + "m") if found else "Not Available", (xcenter - 20, ycenter - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (87, 134, 255), 1)
			
			# Get position of pixel in camera frame
			point3D = rs.rs2_deproject_pixel_to_point(intr, [xcenter, ycenter], float_distance)
			
			# TODO: Use camera orientation to rotate point into fixed frame so we can track object and user in space

		# Render our findings to the screen
		cv2.imshow("color_image preview", color_image)
		if cv2.waitKey(1) == 27:
			break

	pipeline.stop()

if __name__ == '__main__':
	main()
