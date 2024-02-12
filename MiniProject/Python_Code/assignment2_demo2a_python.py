# Assignment 2 Demo Exercise 2b
# Polina Rygina 
# SEED Lab Spring 2024 
# How to Run: Execute using the python terminal. 
# Make sure camera and LCD screen are connected
# Description: 
# Takes a video using the camera and runs ArUco Detection
# If a marker is detected, then it will display a message on the LCD screen
# References: Computer Vision and Communication Tutorial

import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep
import keyboard
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Initialize camera, then pass it into the take photo function
def initializeCamera():
    return cv.VideoCapture(0)

# Takes a photo, with delay to stabilize image
def takePhoto(camera):
    sleep(0.5)
    ret, image = camera.read()
    if not ret:
        print("Could not capture image from camera!")
        quit()
    else:
        conv_img = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        return conv_img

# Runs ArUco detection, sets flag if markers were detected
def arucoDetect(img):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters = cv.aruco.DetectorParameters()
    

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(img)

    if ids is not None:
        detected_marker = True
    else:
        detected_marker = False
        
    return detected_marker, ids, corners

# Prints to LCD a message based on the detected_marker flag.
def printToLCD(conv_img, ids, detected_marker, corners):
    if detected_marker:
        #print("Detected marker IDs: ")
        detected_ids = ', '.join(map(str, ids.flatten()[::-1]))
        #print(detected_ids)
        lcd.message = "Detected Marker:\n" + detected_ids
        cv.aruco.drawDetectedMarkers(conv_img, corners, ids, borderColor = 4)
        sleep(0.5)
        lcd.clear()
        
    else:
        #print("No markers found.")
        lcd.message = "No markers\ndetected."
        sleep(0.5)
        lcd.clear()
        
    return conv_img

if __name__ == "__main__":
    camera = initializeCamera()

    lcd_columns = 16
    lcd_rows = 2
    
    # Initialise I2C bus.
    i2c = board.I2C()  # uses board.SCL and board.SDA
  
    # Initialise the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.color = [0, 100, 100]
    
    # Run loop to continuously take pictures
    while True:
        conv_img = takePhoto(camera)
        conv_img = cv.imread("CdMxo.png")
        detected_markers, ids, corners = arucoDetect(conv_img)
        conv_img = printToLCD(conv_img, ids, detected_markers, corners)

        #Show camera output for debug
        cv.imshow("overlay", conv_img)

        #Exit works only if "overlay" window is selected
        if cv.waitKey(1) & 0xFF == ord('q'):
                break
        
    camera.release()
    cv.destroyAllWindows()
