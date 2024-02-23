# Description of Python Code
## Suplementary Files
- We have two categories of suplemental files:
- - Assignment two files used as a reference: "assigment_2_demo2a_python.py" and "assignment_2_readme.txt", 
they contain alot of the functions used in this project
- - Function prototypes/individual work [everything else] this is due to the fact that we split up the labor
and then combined our functions into one main file
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
