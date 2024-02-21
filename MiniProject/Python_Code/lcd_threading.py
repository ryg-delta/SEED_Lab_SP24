# LCD Threading Functions
# Polina Rygina
# SEED Lab Spring 2024
# How to Run: Execute using the python terminal.
# LCD screen should be connected
# Description:
# References: Computer Vision and Communication Tutorial
# To do: Combine with quadrant detection and improve stabilization

#use cpp coding styles, camelcase for vars, caps for constants

import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading

CAMERA_HEIGHT = 480
CAMERA_WIDTH = 640

lcdMsg = "No marker found."
detectedMarkers = False

def initializeCamera():
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    return camera

# Takes a photo, with delay to stabilize image
def takePhoto(camera):
    sleep(0.1)
    ret, image = camera.read()
    if not ret:
        print("Could not capture image from camera!")
        quit()
    else:
        convImg = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        return convImg


# Runs ArUco detection, sets flag if markers were detected
def arucoDetect(img):
    global lcdMsg
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters = cv.aruco.DetectorParameters()

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(img)

    if ids is not None:
        detectedMarkers = True
        lcdMsg = "Marker detected."
    else:
        detectedMarkers = False
        lcdMsg = "No markers\ndetected."

    return detectedMarkers, ids, corners


# Prints to LCD a message based on the detected_marker flag.
def printToLCD():
    global lcdMsg
    global detectedMarkers
    currentMsg = ""
    while True:
        if lcdMsg != currentMsg:
            lcd.clear()
            if detectedMarkers:
                print("Marker detected.")
                # ******************************
                # Write new data to the LCD here
                # ******************************
                lcd.message = "Marker detected."
                currentMsg = "Marker detected."
            else:
                print("No markers found.")
                lcd.message = "No markers\ndetected."
                currentMsg = "No markers\ndetected."
        sleep(0.1)        
    return




if __name__ == "__main__":
    camera = initializeCamera()

    lcdColumns = 16
    lcdRows = 2

    # Initialise I2C bus.
    i2c = board.I2C()  # uses board.SCL and board.SDA

    # Initialise the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcdColumns, lcdRows)
    lcd.color = [0, 100, 100]

    
    myThread = threading.Thread(target=printToLCD, args=())
    myThread.start()

    # Run loop to continuously take pictures
    while True:
        convImg = takePhoto(camera)
        detectedMarkers, ids, corners = arucoDetect(convImg)

        cv.aruco.drawDetectedMarkers(convImg, corners, ids, borderColor=4)

        # Show camera output for debug
        cv.imshow("overlay", convImg)

        # Exit works only if "overlay" window is selected
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv.destroyAllWindows()

