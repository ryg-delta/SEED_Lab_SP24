# LCD Threading Functions
# Polina Rygina
# SEED Lab Spring 2024
# How to Run: Execute using the python terminal.
# LCD screen should be connected
# Description:
# References: Computer Vision and Communication Tutorial

#use cpp coding styles, camelcase for vars, caps for constants

import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import queue
import threading

q = queue.Queue()

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
        q.put(ids)
    else:
        detected_marker = False

    return detected_marker, ids, corners


# Prints to LCD a message based on the detected_marker flag.
def printToLCD(detected_marker):
    while True:
        if not q.empty():
            if detected_marker:
                ids = q.get()
                print("Detected marker IDs: ")
                detected_ids = ', '.join(map(str, ids.flatten()[::-1]))
                print(detected_ids)

                # ******************************
                # Write new data to the LCD here
                # ******************************
                lcd.message = "Detected Marker:\n" + detected_ids

                #sleep(0.5)
                #lcd.clear()
            else:
                print("No markers found.")
                lcd.message = "No markers\ndetected."
                #sleep(0.5)
                #lcd.clear()
    return




if __name__ == "__main__":
    camera = initializeCamera()

    lcd_columns = 16
    lcd_rows = 2

    # Initialise I2C bus.
    i2c = board.I2C()  # uses board.SCL and board.SDA

    # Initialise the LCD class
    lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
    lcd.color = [0, 100, 100]

    detected_markers = False

    myThread = threading.Thread(target=printToLCD(detected_markers), args=())
    myThread.start()

    # Run loop to continuously take pictures
    while True:
        conv_img = takePhoto(camera)
        detected_markers, ids, corners = arucoDetect(conv_img)

        cv.aruco.drawDetectedMarkers(conv_img, corners, ids, borderColor=4)

        # Show camera output for debug
        cv.imshow("overlay", conv_img)

        # Exit works only if "overlay" window is selected
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv.destroyAllWindows()

