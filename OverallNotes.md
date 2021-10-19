OMISSIONS FROM REPO THAT MUST BE REPLACED:

-   IMU_testing
    -   runIMUTest.sh and runTemp.sh (both follow runnerFormat.txt's format)
-   pyyolo
    -   models -> yolov4-tiny.weights and yolov4.weights
        (download to YOLOv4 main directory and models, then copy out to this director)
    -   runObjectSearch.sh (format text file included)
-   testDataGeneration
    -   runGenerator.sh (format text file included)
-   YOLOv4
    -   Everything EXCEPT:
        -   MakefileFormat.txt
        -   /models: where coco.data, the weights files, and the cfg files for both yolov4 and yolov4-tiny go.
            NOTE: I don't actually remember if I made this folder for organization or if it was built. Might not be needed.

When building YOLOv4, run `make` in the main folder once you're replaced the makefile (I think that's all you need to do).

Extensions I'm using in VSCode for formatting:
Prettier
Pylance
Python

Beyond that, it's up to you.