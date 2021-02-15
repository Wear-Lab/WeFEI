import pyrealsense2 as rs
import numpy as np
import pyyolo
import cv2
import os

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

	while True:
		# Get RealSense frame first so we can guarantee we have one
		frames = pipeline.wait_for_frames()

		# Frames 1280 width x 720 height
		#get_distance(x: int, y: int) -> float
		depth_frame = frames.get_depth_frame()
		color_frame = frames.get_color_frame()

		if not depth_frame or not color_frame:
			continue

		'''
		Apply colormap on depth image (image must be converted to 8-bit per pixel first)
		depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

		# If depth and color resolutions are different, resize color image to match depth image for display
		if depth_colormap_dim != color_colormap_dim:
		    resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
		    images = np.hstack((resized_color_image, depth_colormap))
		else:
		    images = np.hstack((color_image, depth_colormap))

		# Show images
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', images)
		cv2.waitKey(1)
		'''
		
		# Convert images to numpy arrays
		depth_image = np.asanyarray(depth_frame.get_data())
		color_image = np.asanyarray(color_frame.get_data())

		dets = detector.detect(color_image, rgb=False)

		for i, det in enumerate(dets):
			print("Detection: " + str(i) + ", " + str(det))
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

		cv2.imshow("color_image preview", color_image)
		if cv2.waitKey(1) == 27:
			break

	pipeline.stop()

if __name__ == '__main__':
	main()
