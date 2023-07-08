import cv2
import numpy as np
import glob


def save_board_poses(obj_points, img_points, camera_matrix, dist_coeffs):
    objpoints = np.array(obj_points)
    imgpoints = np.array(img_points)
    matrices = []

    for i in range(len(objpoints)):
        ret, rv, tv = cv2.solvePnP(objpoints[i], imgpoints[i], camera_matrix, dist_coeffs)
        rv = cv2.Rodrigues(rv)[0]
        rv = rv.reshape(9)
        tv = tv.reshape(3)
        matrix = np.array([[rv[0], rv[1], rv[2], tv[0]],
                           [rv[3], rv[4], rv[5], tv[1]],
                           [rv[6], rv[7], rv[8], tv[2]],
                           [0, 0, 0, 1]])
        matrices.append(matrix)

    matrices = np.array(matrices)
    np.save('D:/xzFACULTATE/SoSe23/rnm/needle_placement/lab/board_poses.npy', matrices)
    return


# Checkerboard dimensions needed to create the 3D grid of corners
CHECKERBOARD = (8, 5)
SQUARE_SIZE = 0.04  # m
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100000, 1e-5)

# Corners in their nominal positions for single camera calibration
obj_points_rgb = []
img_points_rgb = []

obj_points_depth = []
img_points_depth = []

# Corners in their nominal positions for stereo calibration
obj_points_rgb_stereo = []
img_points_rgb_stereo = []

obj_points_depth_stereo = []
img_points_depth_stereo = []

objp_rgb = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp_rgb[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

objp_depth = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp_depth[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Load images for calibration
rgb_images = sorted(glob.glob('D:/xzFACULTATE/SoSe23/rnm/needle_placement/lab/rgb_images/*.png'))
depth_images = sorted(glob.glob('D:/xzFACULTATE/SoSe23/rnm/needle_placement/lab/ir_images/*.png'))

index = 0
for rgb_image, depth_image in zip(rgb_images, depth_images):

    # Extract RGB image corners
    rgb_img = cv2.imread(rgb_image)
    rgb_image_gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)
    ret_rgb, corners_rgb = cv2.findChessboardCorners(rgb_image_gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH
                                                     + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    # Extract Depth image corners
    depth_img = cv2.imread(depth_image)
    depth_image_gray = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)
    ret_depth, corners_depth = cv2.findChessboardCorners(depth_image_gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH
                                                         + cv2.CALIB_CB_NORMALIZE_IMAGE)

    # Save points for RGB camera calibration
    if ret_rgb:
        obj_points_rgb.append(objp_rgb)
        corners_refined_rgb = cv2.cornerSubPix(rgb_image_gray, corners_rgb, (16, 16), (-1, -1), criteria)
        img_points_rgb.append(corners_refined_rgb)
    else:  # Relevant for the hand-eye calibration
        print(index)

    # Save points for IR camera calibration
    if ret_depth:
        obj_points_depth.append(objp_depth)
        corners_refined_depth = cv2.cornerSubPix(depth_image_gray, corners_depth, (16, 16), (-1, -1), criteria)
        img_points_depth.append(corners_refined_depth)

    # If all corners detected in both images then save them for stereo-calibration.
    if ret_rgb and ret_depth:
        obj_points_rgb_stereo.append(objp_rgb)
        obj_points_depth_stereo.append(objp_depth)
        corners_refined_rgb_stereo = cv2.cornerSubPix(rgb_image_gray, corners_rgb, (16, 16), (-1, -1), criteria)
        img_points_rgb_stereo.append(corners_refined_rgb)
        corners_refined_depth_stereo = cv2.cornerSubPix(depth_image_gray, corners_depth, (16, 16), (-1, -1), criteria)
        img_points_depth_stereo.append(corners_refined_depth)

    index += 1

### Calibrate the cameras using the VALIB_RATIONAL_MODEL flag such that all distortion coefficients can be computed

# Calibrate the RGB cammera and save the board poses for the hand-eye calibration
ret_rgb, camera_matrix_rgb, dist_coeffs_rgb, rvecs_rgb, tvecs_rgb = \
    cv2.calibrateCamera(obj_points_rgb, img_points_rgb, rgb_image_gray.shape[::-1], None, None,
                        flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

print("Camera Matrix RGB:\n", camera_matrix_rgb)

save_board_poses(obj_points_rgb, img_points_rgb, camera_matrix_rgb, dist_coeffs_rgb)

# Calibrate the Depth camera
ret_depth, camera_matrix_depth, dist_coeffs_depth, rvecs_depth, tvecs_depth = \
    cv2.calibrateCamera(obj_points_depth, img_points_depth, depth_image_gray.shape[::-1], None, None,
                        flags=cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

print("Camera Matrix Depth:\n", camera_matrix_depth)

# Stereo calibration with fixed intrinsics
rms, _, _, _, _, R, T, E, F = \
    cv2.stereoCalibrate(obj_points_depth_stereo, img_points_rgb_stereo, img_points_depth_stereo, camera_matrix_rgb, dist_coeffs_rgb,
                        camera_matrix_depth, dist_coeffs_depth, rgb_image_gray.shape[::-1],
                        R=None, T=None, E=None, F=None, flags=cv2.CALIB_FIX_INTRINSIC + cv2.CALIB_RATIONAL_MODEL, criteria=criteria)

print("Rotation Matrix:\n", R)
print("Translation Matrix:\n", T)
print("Essential Matrix:\n", E)
print("Fundamental Matrix:\n", F)
cv2.destroyAllWindows()
