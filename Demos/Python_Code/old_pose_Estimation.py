# Demo 1
# Polina Rygina
# SEED Lab Spring 2024
# How to Run: Execute using the python terminal.
# Make sure either camera is connected for live feed, or you load correct image file.
# Description:
# Runs pose estimation on image, can be extended to video feed.
# References:
# To do: Move axes drawing code to function?


import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep

HEIGHT = 480
WIDTH = 640

X_ORIGIN = WIDTH // 2
Y_ORIGIN = HEIGHT // 2

markerSize = 142

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
    return camera

def arucoDetect(img):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)
    parameters = cv.aruco.DetectorParameters()

    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(img)

    if ids is not None:
        detectedMarkers = True
    else:
        detectedMarkers = False
        lcdMsg = "No markers\ndetected."

    return detectedMarkers, ids, corners

if __name__ == "__main__":
'''
    # read in single image
    img = cv.imread("-20deg.png")
    img = cv.undistort(img, camMtx, distCoeffs)
    detectedMarkers, ids, corners = arucoDetect(img)
    # turn corners into np array
    corners1 = np.array(corners)
    corners1 = corners1.reshape(1, 4, 2)
    topLeft = corners1[0,1]
    topRight = corners1[0,0]
    bottomLeft = corners1[0,3]
    bottomRight = corners1[0,2]
    # draw center of marker
    cX = int((topLeft[0] + topRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomLeft[1]) / 2.0)
    cv.circle(img, (cX, cY), 4, (0,0,255), 2)
    # draw center of image
    cv.circle(img, (X_ORIGIN, Y_ORIGIN),4, (0,0,255), 2)
    

    rvec, tvec, markerPts = my_estimatePoseSingleMarkers(corners, markerSize, camMtx, distCoeffs)
    rvec = np.array(rvec)
    tvec = np.array(tvec)

    # Draw pose using axes.
    cv.drawFrameAxes(img, camMtx, distCoeffs, rvec, tvec, markerSize * 1.5, 2)
    cv.aruco.drawDetectedMarkers(img, corners, ids)
    cv.imshow("target", img)
    cv.waitKey(0)
 '''   
    
    vidCap = initializeCamera()
    sleep(.2)

    # The first frame
    ret, frame = vidCap.read()

    # Show the first frame - ends when '0' is pushed
    cv.imshow('basicImg', frame)
    cv.waitKey(0)
    cv.destroyAllWindows()
    cv.imshow("Live Video", frame)

    # While loop for LIVE video
    
    while vidCap.isOpened():
        # Read in camera data
        ret, frame = vidCap.read()
        grayscale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        img = cv.undistort(grayscale, camMtx, distCoeffs)

        # If the image is good
        if ret:
            # Call aruco detector
            detectedMarkers, ids, corners = arucoDetect(frame)
            # Show the gram
            cv.imshow("Live Video", grayscale)
            # If aruco detected, find the corners
            if detectedMarkers == True:
                corners1 = np.array(corners)
                corners1 = corners1.reshape(1, 4, 2)
                topLeft = corners1[0,1]
                topRight = corners1[0,0]
                bottomLeft = corners1[0,3]
                bottomRight = corners1[0,2]
                cX = int((topLeft[0] + topRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomLeft[1]) / 2.0)
                cv.circle(img, (cX, cY), 4, (0,0,255), 2)

                rvec, tvec, markerPts = my_estimatePoseSingleMarkers(corners, markerSize, camMtx, distCoeffs)
                rvec = np.array(rvec)
                tvec = np.array(tvec)

                # Draw pose using axes.
                cv.drawFrameAxes(img, camMtx, distCoeffs, rvec, tvec, markerSize * 1.5, 2)
                cv.aruco.drawDetectedMarkers(img, corners, ids)

        # wait for 1ms
        cv.waitKey(1)

    # It's over now
    print("Done Now!")
    vidCap.release()
    

    cv.destroyAllWindows()
