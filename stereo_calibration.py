import cv2
import numpy as np
import glob

CHECKERBOARD = (8, 5)
SQUARE_SIZE = 40  # mm
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100000, 0.0001)

obj_points_rgb = []
img_points_rgb = []

obj_points_depth = []
img_points_depth = []

rgb_img_size = (0,0)
ir_img_size = (0,0)

count1 = 0
count2 = 0

objp_rgb = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp_rgb[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

objp_depth = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp_depth[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

rgb_images = glob.glob('/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/rgb_images/*.png')
depth_images = glob.glob('/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/ir_images/*.png')

for rgb_image, depth_image in zip(rgb_images, depth_images):

    # Extract RGB image corners
    rgb_img = cv2.imread(rgb_image)
    rgb_img = cv2.resize(rgb_img, (3840, 2160))
    #rgb_img_size = rgb_image.shape[::-1]
    rgb_image_gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
    ret_rgb, corners_rgb = cv2.findChessboardCorners(rgb_image_gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH
                                                     + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    # Extract Depth image corners
    depth_img = cv2.imread(depth_image)
    depth_img = cv2.resize(depth_img, (640, 576))
    #ir_img_size = depth_img.shape[::-1]
    depth_image_gray = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)
    ret_depth, corners_depth = cv2.findChessboardCorners(depth_image_gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH
                                                         + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if ret_rgb and ret_depth:
        obj_points_rgb.append(objp_rgb)
        obj_points_depth.append(objp_depth)
        corners_refined_rgb = cv2.cornerSubPix(rgb_image_gray, corners_rgb, (16, 16), (-1, -1), criteria)
        img_points_rgb.append(corners_refined_rgb)
        corners_refined_depth = cv2.cornerSubPix(depth_image_gray, corners_depth, (16, 16), (-1, -1), criteria)
        img_points_depth.append(corners_refined_depth)


ret_rgb, camera_matrix_rgb, dist_coeffs_rgb, rvecs_rgb, tvecs_rgb = \
    cv2.calibrateCamera(obj_points_rgb, img_points_rgb, (3840, 2160), None, None,
                        flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

print("Camera Matrix RGB:\n", camera_matrix_rgb)

ret_depth, camera_matrix_depth, dist_coeffs_depth, rvecs_depth, tvecs_depth = \
    cv2.calibrateCamera(obj_points_depth, img_points_depth, (640, 576), None, None,
                        flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

print("Camera Matrix Depth:\n", camera_matrix_depth)

# Stereo calibration

flag, stereoCameraMatrix1, stereoDistCoeffs1, stereoCameraMatrix2, stereoDistCoeffs2, R, T, E, F = \
    cv2.stereoCalibrate(obj_points_depth, img_points_rgb, img_points_depth, camera_matrix_rgb, dist_coeffs_rgb,
                        camera_matrix_depth, dist_coeffs_depth, rgb_image_gray.shape[::-1],
                        R=None, T=None, E=None, F=None, flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

print("Stereo Camera Matrix 1:\n", stereoCameraMatrix1)
print("Stereo Camera Matrix 2:\n", stereoCameraMatrix2)
print("Rotation Matrix:\n", R)
print("Translation Matrix:\n", T)
print("Essential Matrix:\n", E)
print("Fundamental Matrix:\n", F)
cv2.destroyAllWindows()