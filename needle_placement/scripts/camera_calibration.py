#!/usr/bin/env python3
"""Reference: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
"""

import cv2
import numpy as np
import os
import glob
CHECKERBOARD = (8,5)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001)
objpoints = []  #3d point in real world space
imgpoints = []  #2d points in image plane

# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None
images = glob.glob('/home/selva/catkin_ws/src/needle_placement/rosbag_images/*.png')
for fname in images:
    img = cv2.imread(fname)
    img = cv2.resize(img, (2048, 1536)) #with regard to the result.txt file
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret == True:        #if corners are found
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        imgpoints.append(corners2)
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        #savepath = "/home/selva/catkin_ws/src/needle_placement/rosbag_images/calibration_ouput/"
        #cv2.imwrite(os.path.join(savepath , 'calibresult'+str(fname[-5])+'.png'), img)
        #cv2.imshow('img',img)
        #cv2.waitKey(0)

cv2.destroyAllWindows()
h,w = img.shape[:2]
"""Reference to the calibration.txt file, Parameter Count: 14 
   k4, k5, k6 are enabled by setting/calling the flag CALIB_RATIONAL_MODEL (Refer: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)
"""
ret, CameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, flags=cv2.CALIB_RATIONAL_MODEL)
print("Camera matrix : \n")
print(CameraMatrix)  #TODO: check for different number of images
print("dist : \n")
print(distCoeffs)   #TODO: check which are the parameters it is returning (its returning a 2d array)
print("rvecs : \n")
print(rvecs)
print("tvecs : \n")
print(tvecs)

#TODO: Extrinsic parameters