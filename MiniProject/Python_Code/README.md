# Description of Python Code
## Supplementary Files
- We have two categories of suplemental files:
- - Assignment two files used as a reference: "assigment_2_demo2a_python.py" and "assignment_2_readme.txt", 
they contain alot of the functions used in this project
- - Function prototypes/individual work [everything else] this is due to the fact that we split up the labor
and then combined our functions into one main file

## Supplementary File Descriptions
- Detect_Aruco_Quadrants_and_Disp_Video.py: Improves camera feed and implements quadrant detection
- assignment2_demo2a_python.py: Code from Polina's previous assignment which contains ArUco detection functions and video display
- assignment_2_readme.txt: Code from Silje's previous assignment for arduino and python communication along with morphological functions.
- lcd_demo.py: Example code to debug the LCD.
- lcd_threading.py: Implements LCD display via threading

## Main File
- Our main file uses functions to accomplish the goals required by the project
- First, we initalize both the LCD screen, the camera, and i2c arduino interface
- Then, we take a first frame to check to make sure the camera is working, which is exited by presing "0"
- Next, we enter a while loop, which takes in live video from the camera
- Then we process the image (grayscale + dividing into quadrants)
- We call a function to detect aruco markers
- If an aruco marker is detected, we then detect the quadrant of the top-left corner and assign a corresponding integer
- The integer is then sent to both the arduino & the LCD screen
- Program exits when you click the window and press 'q'
