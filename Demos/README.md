# Demo Overview

## Localization and Control
- Goal: Build a framework by which our differential drive robot can sense and navigate its environment.
- Structure: Our main class Robot contains individual packets of functionality that simplify complex tasks into simple commands. Every time a novel function is required, a method is devised and written within that class. This provides easy access to all previously used functionality while simplifying complex tasks by breaking them down into lower units of functionality.

## Computer Vision and Communication
- Goal: Use ArUco Detection to determine the angle of the marker with respect to the camera
- New Functions developed: Camera Calibration / Pose Estimation / Detect Angle
- Interfaces: Used I2C to communicate with LCD to display angle in real time.

## Folders
- Arduino_Code: Contains code for the arduino and control system. 
- Python_Code: Contains computer vision code for Raspberry Pi.
- Matlab_Code: Contains simulink models used to design the position control system.

## Project Board
    status: [none] or [started] or [waiting] or [in progress] or [done]
    Name: Task 1 [status] + task 2 [status] + ... + task n [status] 
 **Silje:** Work on Angle Detection [done] + debugg [done] \
  **Polina:** Work on Charuco [done] + Pose detection [done] + debug [done] \
 **Ben:** Work on simulink [done] + simulations [done] + debugging [done] \
  **Blake:** Work on arduino code for the control system [done] + Encapsulate control libraries [done]
  
