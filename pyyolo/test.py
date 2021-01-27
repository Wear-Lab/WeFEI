import cv2
import pyyolo
import os
from time import time

FPS_COUNTER_MAX = 200
MODEL = "yolov4-tiny"
DATA = "coco"

def main():

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
        
        if fpsMeasureCounter <= FPS_COUNTER_MAX: # To avoid eventual integer overflow, stop counting frames
            fpsMeasureCounter = fpsMeasureCounter + 1

        oldTime = newTime

        for i, det in enumerate(dets):
            if fpsMeasureCounter > FPS_COUNTER_MAX: # Don't print frames if we've stopped counting
                print(f'[Past Frame Count Limit] Detection: {i}, {det}')
            else: # Count frames as it approaches FPS_COUNTER_MAX
                print(f'Frame {fpsMeasureCounter} - Detection: {i}, {det}')
            xmin, ymin, xmax, ymax = det.to_xyxy()
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255))
            cv2.putText(frame, det.name + "," + str(det.prob), (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (107, 168, 50), 1);

        cv2.imshow('Preview Window', frame)
        if cv2.waitKey(1) == 27:
            break

if __name__ == '__main__':
    main()
