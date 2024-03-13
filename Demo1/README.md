# Demo 1 Overview

## Localization and Control
- Goal: Move robot a set amount of feed and rotate robot by a specified amount of degrees.
- Structure: We use control systems to control the angular and radial position of the robot. Both the angular and radial components have position and velocity control systems. These four controllers working in parallel give the robot reasonable responses to angular and radial setpoints.
- Metrics:
  - Average angular dispacement error < 1 deg
  - Average radial displacement error < 1 in

## Computer Vision and Communication
- Goal: Use ArUco Detection to determine the angle of the marker with respect to the camera
- New Functions developed: Camera Calibration / Pose Estimation / Detect Angle
- Interfaces: Used I2C to communicate with LCD to display angle in real time.

## Folders
- Arduino_Code: Contains code for the arduino and control system. 
- Python_Code: Contains computer vision code for Raspberry Pi.
- Matlab_Code: Contains simulink models used to design the position control system.

## Project Board
    -  status: [none] or [started] or [waiting] or [in progress] or [done]
    - Name: Task 1 [status] + task 2 [status] + ... + task n [status] 
 **Silje:** Work on Angle Detection [done] + debugg [done] \
  **Polina:** Work on Charuco [done] + Pose detection [done] + debug [done] \
 **Ben:** Work on simulink [done] + simulations [done] + debugging [done] \
  **Blake:** Work on arduino code for the control system [done] + Encapsulate control libraries [done]
  
