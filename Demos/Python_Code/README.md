# Description of Python Code
## Supplementary Files
- We have three categories of suplemental files:
  - Calibration Utility files: Two folders are dedicated to storing the images used to calibrate each camera as well as their respective camera matrix and distortion coefficient npy files. 
  - Function prototypes/individual work: Since labor was split up, each file contains separate functions.
  - Main: final_main.py, demo2_main.py

## Supplementary File Descriptions
- Angle_Detect_and_Disp_Video_main.py: Runs angle detection algorithm in real time, displaying result to LCD screen.
- calibrate_camera.py: Script that generates camera matrix and distortion coefficient npy files. [REQUIRES opencv_contrib LIBRARY!]
- pose_estimation.py: Shows the pose of each marker, currently configured to work with images instead of video.
- generate_aruco.py: Generates charuco board for camera calibration.
- camOptimization.py: 


## Main File
- Our main file uses functions to accomplish the goals required by the project
- First, we initalize both the LCD screen and the camera
- Then, we take a first frame to check to make sure the camera is working, which is exited by presing "0"
- Next, we enter a while loop, which takes in live video from the camera
- Then we process the image (grayscale + adding center line)
- We call a function to detect aruco markers
- If an aruco marker is detected, calculate the angle between center of the marker and camera
- The angle is then sent to the LCD screen. Negative angle means it is pointed to the right, positive angle means it is pointed to the left.
- Program exits when you click the window and press 'q'
