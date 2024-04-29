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
- camOptimization.py: Test file to improve camera latency
- generate_aruco.py: Generate charuco board for calibrate_camera.py
- Write_I2C_Bits.py: Functions for sending data to the Arduino with as minimal bytes as possible

## Main File
- Runs aruco detection and pose estimation to get the distance and angle of any markers in view
- Sends distance and angle to Arduino to determine location of markers.
- Camera Optimizations implemented to prevent over exposure
