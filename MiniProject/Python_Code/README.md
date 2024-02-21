# Coding Style Quick Ref
## Use C style conventions 
- Constants: `CONSTANT`
- Variables: `camelCase`
- Functions: `camelCase`
- Import names:
  - `import cv2 as cv`
  - `import numpy as np`
- Use: Main and Function structure for easy debug
- Use: `# Name of file`
       `# Author name(s)`
       `# SEED Lab Spring 2024`
       `# How to Run:`
       `# Description:`
       `# References:`
# RPI and Arduino Interfacing
- SCL: A5 to Pin 5
- SDA: A4 to Pin 3
- GND: GND to Pin 6

# Description of Python Code
## Suplementary Files
- We have two categories of suplemental files:
- - Assignment two files used as a reference: "assigment_2_demo2a_python.py" and "assignment_2_readme.txt", 
they contain alot of the functions used in this project
-- Function prototypes/individual work [everything else] this is due to the fact that we split up the labor
and then combined our functions into one main file
## Main File
- Our main file uses functions to accomplish the goals required by the project
- First, we initalize both the LCD screen, the camera, and i2c arduino interface
- Then, we take a first frame to check to make sure the camera is working, which is exited by presing "0"
- Next, we enter a while loop, which takes in live video from the camera
- Then we process the image (grayscale + dividing into quadrants)
- We call a function to detect aruco markers
- If an aruco marker is detected, we then detect the quadrant of the top-left corner
- This is then sent to both the arduino & the lcd screen
- It is exited when you click the window and press q
