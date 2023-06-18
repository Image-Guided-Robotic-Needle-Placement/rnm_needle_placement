import cv2
import numpy as np
import glob

CHECKERBOARD = (8,5)
SQUARE_SIZE = 0.04
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 1000, 1e-6)

objpoints_rgb = []
imgpoints_rgb = []

objpoints_depth = []
imgpoints_depth = []

count1 = 0
count2 = 0

objp_rgb = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp_rgb[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

objp_depth = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp_depth[:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

rgb_images = glob.glob('../../rosbag_images/*.png')
for fname in rgb_images:
    rgb_img = cv2.imread(fname)
    rgb_img = cv2.resize(rgb_img, (2048, 1536))
    gray1 = cv2.cvtColor(rgb_img,cv2.COLOR_BGR2GRAY)
    ret1, corners1 = cv2.findChessboardCorners(gray1, CHECKERBOARD,cv2.CALIB_CB_ADAPTIVE_THRESH)
    if ret1 == True and count1 < 25:
        count1+=1
        objpoints_rgb.append(objp_rgb)
        corners_1 = cv2.cornerSubPix(gray1, corners1, (16,16),(-1,-1), criteria)
        imgpoints_rgb.append(corners_1)
        rgb_img = cv2.drawChessboardCorners(rgb_img, CHECKERBOARD, corners_1, ret1)
        #cv2.imshow('img',rgb_img)  
        #cv2.waitKey(0)

cv2.destroyAllWindows()
ret1, CameraMatrix1, distCoeffs1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints_rgb, imgpoints_rgb, 
                                                                       gray1.shape[::-1], None, None, 
                                                                       flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

print("Camera Matrix:\n", CameraMatrix1)

depth_images = glob.glob('../../rosbag_images/depth_images/*.png')
for fname in depth_images:
    #print(fname)
    depth_img = cv2.imread(fname)
    depth_img = cv2.resize(depth_img, (640, 576))
    gray2 = cv2.cvtColor(depth_img,cv2.COLOR_BGR2GRAY)
    ret2, corners2 = cv2.findChessboardCorners(gray2, CHECKERBOARD,cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if ret2 == True and count2 < 25:
        #print(fname+" found")
        count2+=1
        objpoints_depth.append(objp_depth)
        corners_2 = cv2.cornerSubPix(gray2, corners2, (16,16),(-1,-1), criteria)
        imgpoints_depth.append(corners_2)
        depth_img = cv2.drawChessboardCorners(depth_img, CHECKERBOARD, corners_2, ret2)
        #cv2.imshow('img',depth_img)  
        #cv2.waitKey(0)

cv2.destroyAllWindows()
print(count1, count2)

ret2, CameraMatrix2, distCoeffs2, rvecs2, tvecs2 = cv2.calibrateCamera(objpoints_depth, imgpoints_depth,
                                                                       gray2.shape[::-1], None, None, 
                                                                       flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)
print("Camera Matrix:\n", CameraMatrix2)

#stereo calibration

flag, stereoCameraMatrix1, stereoDistCoeffs1, stereoCameraMatrix2, stereoDistCoeffs2, R, T, E, F = cv2.stereoCalibrate(objpoints_depth, 
                                                                                                                       imgpoints_rgb, imgpoints_depth, 
                                                                                                                       CameraMatrix1, distCoeffs1, CameraMatrix2, distCoeffs2, 
                                                                                                                       gray1.shape[::-1], R=None, T=None, E=None, F=None,
                                                                                                                       flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)
print("Stereo Camera Matrix 1:\n", stereoCameraMatrix1)
print("Stereo Camera Matrix 2:\n", stereoCameraMatrix2)
print("Rotation Matrix:\n", R)
print("Translation Matrix:\n", T)
print("Essential Matrix:\n", E)
print("Fundamental Matrix:\n", F)
