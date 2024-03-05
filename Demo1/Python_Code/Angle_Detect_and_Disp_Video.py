# Demo 1
# Polina Rygina and Silje Ostrem
# SEED Lab Spring 2024
# How to Run: Execute using the python terminal.
# Make sure camera and LCD screen are connected
# Description:
#
# References:
# To do:
# Similar Triangles Distance Formula
# ArUco marker dimensions (Potentially transformation)
# Difference of height = angle?

import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep
import board
# import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import threading
import math

# Constants
HEIGHT = 480
WIDTH = 640

X_ORIGIN = WIDTH // 2
Y_ORIGIN = HEIGHT // 2

FOCAL_LENGTH = 120  # mm

KNOWN_MARKER_WIDTH = 50  # mm
KNOWN_MARKER_LENGTH = 50  # mm

HFOV = 56.88  # deg

lcdMsg = "No markers\ndetected."
detectedMarkers = False

detectedHeight = 0
detectedWidth = 0

actualWidth = 0
actualHeight = 0

detectedAngle = 0

distanceFromCenter = 0
distance = 0
# topLeft, top right, bottom right, bottom left
cornerCoors = [[0, 0], [0, 0], [0, 0], [0, 0]]


def initializeCamera():
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)
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
    global detectedCenter
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters = cv.aruco.DetectorParameters()

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(img)

    if ids is not None:
        detectedMarkers = True
        # Assigns Corner COordinates
        cornerCoors = [[corners[0][0][0][0], corners[0][0][0][1]],
                       [corners[0][0][1][0], corners[0][0][1][1]],
                       [corners[0][0][2][0], corners[0][0][2][1]],
                       [corners[0][0][3][0], corners[0][0][3][1]]]
        detectedCenter = [cornerCoors[0][0] + (cornerCoors[1][0] - cornerCoors[0][0]) / 2,
                          cornerCoors[3][1] + (cornerCoors[0][1] - cornerCoors[3][1])]
    else:
        detectedMarkers = False
        lcdMsg = "No markers\ndetected."

    return detectedMarkers, ids, corners


# Prints to LCD a message based on the detected_marker flag.
'''
def printToLCD():
    global lcdMsg
    global detectedMarkers
    currentMsg = ""
    while True:
        if lcdMsg != currentMsg:
            lcd.clear()
            if detectedMarkers:
                # print("Marker detected.")
                # ******************************
                # Write new data to the LCD here
                # ******************************
                lcd.message = lcdMsg
                currentMsg = lcdMsg
            else:
                # print("No markers found.")
                lcd.message = "No markers\ndetected."
                currentMsg = "No markers\ndetected."
        sleep(0.1)
    return
'''


def actualHeightAndWidth():
    # diff in y coordinates from top corners
    diff_height = abs(cornerCoors[0][1] - cornerCoors[1][0])
    # diff in x coordinates from left corners
    diff_width = abs(cornerCoors[0][0] - cornerCoors[3][0])
    # Pythagoras
    actualWidth = math.sqrt(diff_height ** 2 + diff_width ** 2)
    actualLength = math.sqrt(diff_width ** 2 + detectedHeight ** 2)


def angle_detect():
    # use similar triangles
    distanceFromCenter = detectedCenter[0] - X_ORIGIN
    # Convert from pixels to mm
    '''
    distanceFromCenter = distanceFromCenter*(KNOWN_MARKER_WIDTH/actualWidth)
    angle = math.tan(distanceFromCenter/distance)
    #Convert from radians
    angle = angle *180/math.pi
    '''
    angle = HFOV * (distanceFromCenter / (WIDTH))
    print("distance from center: ", distanceFromCenter)
    return angle


def distance():
    distance = KNOWN_MARKER_WIDTH * FOCAL_LENGTH / actualHeight
    # Return the distance from the camera
    return distance


def mm_to_in(var):
    var = var / 25.4
    return var


if __name__ == "__main__":
    # lcdColumns = 16
    # lcdRows = 2

    # Initialise I2C bus.
    # i2c = board.I2C()  # uses board.SCL and board.SDA
    sleep(1)

    # Initialise the LCD class
    # lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcdColumns, lcdRows)
    # lcd.color = [0, 100, 100]

    # myThread = threading.Thread(target=printToLCD, args=())
    # myThread.start()

    vidCap = initializeCamera()
    sleep(.2)

    # The first frame
    ret, frame = vidCap.read()

    # Show the first frame - ends when '0' is pushed
    cv.imshow('basicImg', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()
    cv.imshow("Live Video", frame)

    camMtx = np.load("camera_matrix.npy")
    distCoeffs = np.load("dist_coeffs.npy")

    # While loop for LIVE video
    while vidCap.isOpened():
        # Read in camera data
        ret, frame = vidCap.read()
        grayscale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        undistorted_image = cv.undistort(grayscale, camMtx, distCoeffs)

        # If the image is good
        if ret:
            # Add lines to the image & Make it grayescale

            cv.line(grayscale, (X_ORIGIN, 0), (X_ORIGIN, HEIGHT), color=(0, 255, 0), thickness=5)
            # Call aruco detector
            detectedMarkers, ids, corners = arucoDetect(frame)
            # Show the gram
            cv.imshow("Live Video", grayscale)
            # If aruco detected, find the corners
            if detectedMarkers == True:
                lcdMsg = "Aruco Detected"
                print(lcdMsg)
                angle = angle_detect()
                lcdMsg = f"Angle is: {angle}"
                print("Center of Marker: ", detectedCenter)
                print(lcdMsg)

        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        # wait for 1ms
        cv.waitKey(1)

    # It's over now
    print("Done Now!")
    vidCap.release()
    cv.destroyAllWindows()