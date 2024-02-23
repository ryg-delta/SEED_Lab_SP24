# Mini Project Overview
Goal: Position wheels depending on the quadrant the ArUco marker is detected in. LCD screen displays desired goal position.
- NE: 0 0
- NW: 0 1
- SE: 1 0
- SW: 1 1

## Localization and Control
Goal: Position wheels to either a `1` or `0` position based on instructions from the RPi.

Structure: The Arduino recieves an I2C byte with the two LSBs corresponding to the left and right wheel position targets accordingly. A PI controller is then used to turn the wheel to the desired position.

Metrics: The target position will update as soon as a message is recieved. The average rise time of the system is about `0.4` Sec.

## Computer Vision and Communication
- Goal: Use ArUco Detection to determine which quadrant the marker is in and send an integer to the Arduino to control motors.
- New Functions developed: Threading with LCD / ArUco Quadrant Detection / Improved Camera Feed
- Interfaces: Used I2C to send and integer to the Arduino which would then be interpreted as the respective binary number. I2C was also used to display text to the LCD, only updating when the quadrant changes.
