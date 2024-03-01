import cv2 as cv
import numpy as np
from time import sleep
#OpenCV Contrib Vers!
aruco_dict = cv.aruco.DICT_6X6_50
ROWS = 7
COLS = 5
SQUARE_LENGTH = 0.03
MARKER_LENGTH = 0.015
#other calibrated cam values: https://github.com/PhotonVision/photonvision/blob/7f09f9e4f5b4237ef4b9dde7fdcb747115315659/photon-server/src/main/resources/calibration/lifecam480p.json#L10
#https://mecaruco2.readthedocs.io/en/latest/notebooks_rst/Aruco/sandbox/ludovic/aruco_calibration_rotation.html
def calibrate():
    dictionary = cv.aruco.getPredefinedDictionary(aruco_dict)
    board = cv.aruco.CharucoBoard((ROWS, COLS), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    parameters = cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)
    charucoCorners = []
    charucoIds = []
    images = ["0.png", "1.png", "2.png", "3.png", "4.png", "5.png", "6.png", "7.png", "8.png", "9.png"]


    for image in images:
        img = cv.imread(image)
        img = img.copy()
        corners, ids, rejected = detector.detectMarkers(img)

        #cv.aruco.drawDetectedMarkers(img, corners, ids)

        charucodetector = cv.aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, corners, ids = charucodetector.detectBoard(img)
        cv.aruco.drawDetectedCornersCharuco(img, charuco_corners, charuco_ids)
        cv.imshow('Undistorted Image', img)
        cv.waitKey(0)


        charucoCorners.append(charuco_corners)
        charucoIds.append(charuco_ids)

    imsize = img.shape
    # prolly overkill but can help tweak
    cameraMatrixInit = np.array([[333., 0., imsize[0] / 2.],
                                 [0., 333., imsize[1] / 2.],
                                 [0., 0., 1.]])

    distCoeffsInit = np.zeros((5, 1))

    flags = (cv.CALIB_USE_INTRINSIC_GUESS + cv.CALIB_RATIONAL_MODEL + cv.CALIB_FIX_ASPECT_RATIO)
    #flags = (cv.CALIB_RATIONAL_MODEL) causes mega distortion lmao
    (ret, camMtx, distCoeffs,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv.aruco.calibrateCameraCharucoExtended(
        charucoCorners=charucoCorners,
        charucoIds=charucoIds,
        board=board,
        imageSize=imsize[:2],
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv.TERM_CRITERIA_EPS & cv.TERM_CRITERIA_COUNT, 10000, 1e-9))


    print(camMtx)
    print(distCoeffs)
    # Save calibration data
    np.save('camera_matrix.npy', camMtx)
    np.save('dist_coeffs.npy', distCoeffs)

    # Iterate through displaying all the images
    for image in images:
        img = cv.imread(image)
        undistorted_image = cv.undistort(img, camMtx, distCoeffs)
        cv.imshow('Undistorted Image', undistorted_image)
        cv.waitKey(0)

    cv.destroyAllWindows()

calibrate()

