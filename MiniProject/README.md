# Mini Project Overview
Goal: Position wheels depending on the quadrant the ArUco marker is detected in. LCD screen displays desired goal position.
- NE: 0 0
- NW: 0 1
- SE: 1 0
- SW: 1 1

## Localization and Control

## Computer Vision and Communication
- Goal: Use ArUco Detection to determine which quadrant the marker is in and send an integer to the Arduino to control motors.
- New Functions developed: Threading with LCD / ArUco Quadrant Detection / Improved Camera Feed
- Interfaces: Used I2C to send and integer to the Arduino which would then be interpreted as the respective binary number. I2C was also used to display text to the LCD, only updating when the quadrant changes.
