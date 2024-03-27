# Demo 2
# Polina Rygina
# SEED Lab Spring 2024
# How to Run: Execute using the python terminal.
# Make sure either camera is connected for live feed. Camera coefficients need to be in the same
# directory as script.
# Description:
# Runs pose estimation video feed. DEPENDENT ON MARKER SIZE FOR ACCURACY
# References:
# To do:

import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep
import imageio

HEIGHT = 480
WIDTH = 640

X_ORIGIN = WIDTH // 2
Y_ORIGIN = HEIGHT // 2

#ENSURE THIS IS ACCURATE
markerSize = 138

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
    return detectedMarkers, ids, corners

def pixToMeter(pix):
    meters = pix * 0.0002645833
    return meters


if __name__ == "__main__":

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

            # If aruco detected, find the corners
            if detectedMarkers == True:
                totalMarkers = range(0, ids.size)
                for ids, corner, i in zip(corners, ids, totalMarkers):
                    rvec = np.array(rvec)
                    tvec = np.array(tvec)
                    print(tvec)
                    #dist = tvec[i][2]
                    dist = np.sqrt(tvec[i][2] ** 2 + tvec[i][0] ** 2 + tvec[i][1] ** 2)
                    dist = pixToMeter(dist)

                    topRight = corners[i][0][0]
                    topRight = (int(topRight[0]), int(topRight[1]))

                    # Draw pose using axes.
                    cv.drawFrameAxes(img, camMtx, distCoeffs, rvec[i], tvec[i], markerSize * 0.7, 2)
                    # Draw distance in real time
                    cv.putText(
                        img,
                        f"Dist: {dist}",
                        topRight,
                        cv.FONT_HERSHEY_PLAIN,
                        1.0,
                        (0, 0, 255),
                        2,
                        cv.LINE_AA,
                    )

            cv.imshow("Live Video", img)
            key = cv.waitKey(1)
            if key == ord("a"):
                cv.imwrite("pose_estimate.png", img)
            elif key == ord("q"):
                break


print("Done Now!")
vidCap.release()

cv.destroyAllWindows()