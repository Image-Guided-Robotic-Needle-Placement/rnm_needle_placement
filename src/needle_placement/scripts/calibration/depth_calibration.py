#!/usr/bin/env python3
"""Reference: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
"""

import cv2
import numpy as np
import glob

CHECKERBOARD = (8, 5)
SQUARE_SIZE = 40  # mm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-6)
obj_points = []  # 3D point in real world space
img_points = []  # 2D points in image plane

# Defining the world coordinates for 3D points
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE
images = glob.glob('../../rosbag_images/depth_images/*.png')

for image in images:
    img = cv2.imread(image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret:        # if corners are found
        obj_points.append(objp)
        # refining pixel coordinates for given 2d points.
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        img_points.append(corners_refined)

cv2.destroyAllWindows()

h, w = img.shape[:2]
"""Reference to the calibration.txt file, Parameter Count: 14 
   k4, k5, k6 are enabled by setting/calling the flag CALIB_RATIONAL_MODEL (Refer: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d)
"""
# Intrinsic matrix
ret, CameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None, flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

# Error
mean_error = 0
for i in range(len(obj_points)):
    reprojected_points, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], CameraMatrix, distCoeffs)
    error = cv2.norm(img_points[i], reprojected_points, cv2.NORM_L2) / len(reprojected_points)
    mean_error += error

# Write the calibration results
with open('./depth_camera_calibration.txt', 'w') as f:
    f.write('Camera Matrix:\n')
    f.write(np.array2string(CameraMatrix, precision=3))
    f.write('\n')
    f.write('Distortion Coefficients:\n')
    f.write(np.array2string(distCoeffs, precision=6))

print("Camera matrix : \n")
print(CameraMatrix)
print("dist : \n")
print(distCoeffs)   
print("total error: ", mean_error / len(obj_points))
