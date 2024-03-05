import cv2 as cv
from cv2 import aruco
import numpy as np
from time import sleep

# Calculating Yaw-axis angle of rotation
HEIGHT = 480
WIDTH = 640

X_ORIGIN = WIDTH // 2
Y_ORIGIN = HEIGHT // 2

camMtx = np.load("camera_matrix.npy")
distCoeffs = np.load("dist_coeffs.npy")

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
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
    img = cv.imread("-20deg.png")
    detectedMarkers, ids, corners = arucoDetect(img)
    img = cv.undistort(img, camMtx, distCoeffs)
    print(detectedMarkers)
    print(ids)
    print(corners)
    corners1 = np.array(corners)
    corners1 = corners1.reshape(1, 4, 2)
    topLeft = corners1[0,1]
    topRight = corners1[0,0]
    bottomLeft = corners1[0,3]
    bottomRight = corners1[0,2]
    cX = int((topLeft[0] + topRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomLeft[1]) / 2.0)
    cv.circle(img, (cX, cY), 4, (0,0,255), 2)

    cv.circle(img, (X_ORIGIN, Y_ORIGIN),4, (0,0,255), 2)
    print(cX)
    print(cY)
    markerSize = 142
    rvec, tvec, markerPts = my_estimatePoseSingleMarkers(corners, markerSize, camMtx, distCoeffs)
    rvec = np.array(rvec)
    tvec = np.array(tvec)
    cv.drawFrameAxes(img, camMtx, distCoeffs, rvec, tvec, markerSize * 1.5, 2)
    cv.aruco.drawDetectedMarkers(img, corners, ids)
    cv.imshow("target", img)
    cv.waitKey(0)

    #vidCap = initializeCamera()
    #sleep(.2)

    # The first frame
    #ret, frame = vidCap.read()

    # Show the first frame - ends when '0' is pushed
    #cv.imshow('basicImg', frame)
    #cv.waitKey(0)
    #cv.destroyAllWindows()
    #cv.imshow("Live Video", frame)

    # While loop for LIVE video
    '''
    while vidCap.isOpened():
        # Read in camera data
        ret, frame = vidCap.read()
        grayscale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        # If the image is good
        if ret:
            # Call aruco detector
            detectedMarkers, ids, corners = arucoDetect(frame)
            # Show the gram
            cv.imshow("Live Video", grayscale)
            # If aruco detected, find the corners
            if detectedMarkers == True:
                sleep(0.1)

        # wait for 1ms
        cv.waitKey(1)

    # It's over now
    print("Done Now!")
    vidCap.release()
    '''

    cv.destroyAllWindows()