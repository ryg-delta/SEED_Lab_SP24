# Demo 2
# Polina Rygina
# SEED Lab Spring 2024
# How to Run: Execute using the python terminal.
# Make sure either camera is connected for live feed. Camera coefficients need to be in the same
# directory as script.
# Description:
# Runs pose estimation video feed. DEPENDENT ON MARKER SIZE FOR ACCURACY
# References:
# To do: Add find closest marker function

import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep
import board
import threading
import time
from smbus2 import SMBus


HEIGHT = 480
WIDTH = 640

X_ORIGIN = WIDTH // 2
Y_ORIGIN = HEIGHT // 2

#ENSURE THIS IS ACCURATE
markerSize = 3.8 * 37.7952755906 # cm * factor

ARD_ADDR = 0x08
HFOV = 60  # deg

# Flags and other global variables

detectedMarkers = False

detectedHeight = 0
detectedWidth = 0

actualWidth = 0
actualHeight = 0

angle = 0.0

distanceFromCenter = 0
distance = 0
# topLeft, top right, bottom right, bottom left
cornerCoors = [[0, 0], [0, 0], [0, 0], [0, 0]]

camMtx = np.load("camera_matrix.npy")
distCoeffs = np.load("dist_coeffs.npy")


# Returns pose of the marker in the form of rotation and translation vectors.
def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    for c in corners:
        nada, R, t = cv.solvePnP(marker_points, c, mtx, distortion, False, cv.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def initializeCamera():
    camera = cv.VideoCapture(0)
    camera.set(cv.CAP_PROP_FRAME_WIDTH, WIDTH)
    camera.set(cv.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    camera.set(cv.CAP_PROP_AUTO_EXPOSURE, 3)
    camera.set(cv.CAP_PROP_AUTO_EXPOSURE, 1)
    camera.set(cv.CAP_PROP_BRIGHTNESS, 250)
    camera.set(cv.CAP_PROP_EXPOSURE, 39)
    camera.set(cv.CAP_PROP_BUFFERSIZE, 1)
    camera.set(cv.CAP_PROP_FPS, 120)
    return camera


# Runs ArUco detection, sets flag if markers were detected
def arucoDetect(img):
    global lcdMsg
    global detectedCenter
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters = cv.aruco.DetectorParameters()
    parameters.minMarkerPerimeterRate = 0.01

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(img)

    if ids is not None:
        detectedMarkers = True
        # Assigns Corner Coordinates
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


def angle_detect():
    # use similar triangles
    distanceFromCenter = detectedCenter[0] - X_ORIGIN
    # Convert from pixels to mm
    angle = HFOV * (distanceFromCenter / (WIDTH))
    #print("distance from center: ", distanceFromCenter)
    return angle

def pixToMeter(pix):
    meters = pix * 0.0002645833
    return meters

def find_closest(distance):
    closest = np.min(distance)
    closest_idx = np.where(distance==closest[0][0])
    return closest

def write_data(angle, distance):
    if detectedMarkers:
        angle_sent = (-angle+HFOV/2)*(255/HFOV)
        angle_sent = int(angle_sent)
        distance = int(np.round(distance))
        try:
            ARD_i2c.write_byte_data(ARD_ADDR, angle_sent, distance)
            print("angle:",angle_sent)
            print("cm:", distance)
        except OSError:
            print("Unable to write.")

if __name__ == "__main__":
    
    ARD_i2c = SMBus(1)
    sleep(1)

    vidCap = initializeCamera()
    sleep(1)

    # The first frame
    ret, frame = vidCap.read()

    # Show the first frame - ends when '0' is pushed
    cv.imshow('basicImg', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()
    cv.imshow("Live Video", frame)
    detectedMarkers = False
    
    # While loop for LIVE video

    while True:
        # Read in camera data
        ret, img = vidCap.read()
        img = cv.undistort(img, camMtx, distCoeffs)
        
        # If the image is good
        frames = []
        imgCount = 0
        if ret:
            # Call aruco detector
            detectedMarkers, ids, corners = arucoDetect(img)
            rvec, tvec, markerPts = my_estimatePoseSingleMarkers(corners, markerSize, camMtx, distCoeffs)
            prevState = detectedMarkers
            #distance = np.empty(6)
            # If aruco detected, find the corners
            if detectedMarkers == True:
                if prevState != True:
                    write_data(0, 0)
                totalMarkers = range(0, ids.size)
                for ids, corner, i in zip(corners, ids, totalMarkers):
                    rvec = np.array(rvec)
                    tvec = np.array(tvec)

                    dist = np.sqrt(tvec[i][2] ** 2 + tvec[i][0] ** 2 + tvec[i][1] ** 2)
                    distance = np.round(pixToMeter(dist),decimals=2) * 95
                    #distance[i] = calcDistance
                    angle = -angle_detect() + 3.15

                #sendDistance = find_closest(distance)
                write_data(angle, distance)

            #cv.imshow("Live Video", img)
            
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
            # wait for 1ms
        

print("Done Now!")
vidCap.release()
cv.destroyAllWindows()

