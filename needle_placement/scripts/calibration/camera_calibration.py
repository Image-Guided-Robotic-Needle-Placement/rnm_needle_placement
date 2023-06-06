#!/usr/bin/env python3
"""Reference: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
"""

import cv2
import numpy as np
import os
import glob
CHECKERBOARD = (8, 5)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-5)
objpoints = []  #3d point in real world space
imgpoints = []  #2d points in image plane

# Defining the world coordinates for 3D points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
images = glob.glob('../../images/*.png')
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
        #savepath = "../rosbag_images/calibration_ouput/"
        #cv2.imwrite(os.path.join(savepath , 'calibresult'+str(fname[-5])+'.png'), img)
        #cv2.imshow('img',img)
        #cv2.waitKey(0)

cv2.destroyAllWindows()
h,w = img.shape[:2]
"""Reference to the calibration.txt file, Parameter Count: 14 
   k4, k5, k6 are enabled by setting/calling the flag CALIB_RATIONAL_MODEL (Refer: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)
"""
#Intrinsic matrix
ret, CameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None, flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

#error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], CameraMatrix, distCoeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

#writing the calibration results
with open('./lab_intrinsic_result.txt', 'w') as f:
    f.write('Camera Matrix:\n')
    f.write(np.array2string(CameraMatrix, precision=5))
    f.write('\n')
    f.write('Distortion Coefficients:\n')
    f.write(np.array2string(distCoeffs, precision=6))

print("Camera matrix : \n")
print(CameraMatrix)
print("dist : \n")
print(distCoeffs)   
print( "total error: {}".format(mean_error/len(objpoints)))

#Extrinsic (if needed)
objpoints = np.array(objpoints)
imgpoints = np.array(imgpoints)
with open('./lab_extrinsic_result.txt', 'w') as f:
    for i in range(len(objpoints)):
        ret, rvecs, tvecs = cv2.solvePnP(objpoints[i], imgpoints[i], CameraMatrix, distCoeffs) #(https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)
        rvecs = cv2.Rodrigues(rvecs)[0]
        rvecs = rvecs.reshape(9)
        tvecs = tvecs.reshape(3)
        matrix = np.array([[rvecs[0], rvecs[1], rvecs[2], tvecs[0]],
                                [rvecs[3], rvecs[4], rvecs[5], tvecs[1]],
                                [rvecs[6], rvecs[7], rvecs[8], tvecs[2]],
                                [0, 0, 0, 1]])
        f.write('Image ' + str(i+1) + ':\n')
        f.write(np.array2string(matrix, precision=6))
        f.write('\n')
        f.write('\n')