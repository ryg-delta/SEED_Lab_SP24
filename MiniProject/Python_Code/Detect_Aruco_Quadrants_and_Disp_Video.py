 #Silje Ostrem
#EENG 350 Mini Project

# The goal of this code is to detect aruco markers
#Then tell which quadrant they are in
#And report that to the main

#Hardware Connections: USB Webcam

#We are using our style guide

import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep

HEIGHT = 480
WIDTH = 640



#detect aruco

def arucoDetect(img):
	#This detects the aruco markers and returns their parameters
    sleep(.1)
	#Dictionary 6x6 50
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters = cv.aruco.DetectorParameters()
	#This function calls the built in aruco detector
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(img)
	#Checks if any marker is detectedd
    if ids is not None:
        detected_marker = True
    else:
        detected_marker = False
        print("NO Marker Detected")
	#Returns findings
    return detected_marker, ids, corners

#Detect the quadarant the aruco marker is in
def detectQuad(corners):
	#Gets coordinates from the aruco detector
    xCoor = corners[0][0][0][0]
    yCoor = corners[0][0][0][1]
	#Each quadrante is labeled 0-4, [00, 01][10,11] (in binary)
    pos = 0
	#adds the appropriate ammount based on position
    if xCoor >= halfWidth:
        pos += 1
    if yCoor >= halfHeight:
        pos += 2
	#Returns the final quadrant
    return(pos)


#Capture Video
vidCap = cv.VideoCapture(0)
sleep(.2)



#The first frame
ret, frame = vidCap.read()

#Height + Width
halfWidth = WIDTH//2
halfHeight = HEIGHT//2


#Show the first frame - ends when '0' is pushed

cv.imshow('basicImg', frame)
cv.waitKey(0)
cv.destroyAllWindows()
cv.imshow("Live Video", frame)
#Initializations of the positons
pos = 5
oldpos = 5
#While loop for LIVE video
while vidCap.isOpened():
	#Read in camera data
    ret,  frame = vidCap.read()  
    grayscale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

	#If the image is good
    if ret:
        #Add lines to the image & Make it grayescale
        cv.line(grayscale, (0,halfHeight),(WIDTH, halfHeight), (0,255,0), thickness=5)
        cv.line(grayscale,(halfWidth,0),(halfWidth, HEIGHT), color=(0,255,0), thickness=5)
	#Call aruco detector       
	 markers, ids, corners = arucoDetect(frame)
	#Show the gram
        cv.imshow("Live Video", grayscale)
    	#If aruco detected, find the corners
        if markers == True:
            pos = detectQuad(corners)
            #If the positon has changed, print
            if oldpos != pos:
                 oldpos = pos
                 print(pos)
	#wait for 1ms
    cv.waitKey(1)

#It's over now
print("Done NOw!")
vidCap.release()
