# Demo 1 Overview

## Localization and Control
-Goal: Move robot a set amount of feed and rotate robot by a specified amount of degrees.
-Structure:
-Metrics: 

## Computer Vision and Communication
- Goal: Use ArUco Detection to determine the angle of the marker with respect to the camera
- New Functions developed: Camera Calibration / Pose Estimation / Detect Angle
- Interfaces: Used I2C to communicate with LCD to display angle in real time.

## Folders
- Arduino_Code: Contains code for the arduino and control system. 
- Python_Code: Contains computer vision code for Raspberry Pi.
- Matlab_Code: Contains simulink models used to design the position control system.
