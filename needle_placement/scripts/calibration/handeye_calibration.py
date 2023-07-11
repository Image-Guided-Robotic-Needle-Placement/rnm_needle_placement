import numpy as np
import cv2


def read_poses_from_npy(path):
    poses = np.load(path)
    translations = []
    rotations = []

    for pose in poses:
        translations.append(np.array(pose[0:3, 3]))
        rotations.append(np.array(pose[0:3, 0:3]))

    return translations, rotations, np.array(poses)


def compute_calibration_rmse(endeffector_poses, camera_poses, handeye_transform):
    rmse = 0
    board_pose_mean = np.zeros((4, 4))

    for end_pose, cam_pose in zip(endeffector_poses, camera_poses):
        board_pose_mean = board_pose_mean + np.matmul(end_pose, np.matmul(handeye_transform, cam_pose))

    board_pose_mean /= len(endeffector_poses)

    for end_pose, cam_pose in zip(endeffector_poses, camera_poses):
        board_pose_translational_error = board_pose_mean[0:3, 3] - np.matmul(end_pose, np.matmul(handeye_transform, cam_pose))[0:3, 3]
        rmse += np.linalg.norm(board_pose_translational_error)

    return np.sqrt(rmse / (3 * len(endeffector_poses)))

endeffector_poses_path = 'D:/xzFACULTATE/SoSe23/rnm/rnm_needle_placement-lab-final-fr/rnm_needle_placement-lab-final-fr/src/needle_placement/lab/endeffector_poses.npy'
board_poses_path = 'D:/xzFACULTATE/SoSe23/rnm/needle_placement/lab/board_poses.npy'

# Load poses from files
endeffector_translations, endeffector_rotations, endeffector_poses = read_poses_from_npy(endeffector_poses_path)
camera_translations, camera_rotations, camera_poses = read_poses_from_npy(board_poses_path)

# Do the hand-eye calibration which finds the transformation camera -> end-effector
handeye_rotation, handeye_translation = cv2.calibrateHandEye(endeffector_rotations, endeffector_translations, camera_rotations, camera_translations)
handeye_transform = np.eye(4)
handeye_transform[0:3, 0:3] = handeye_rotation
handeye_transform[0:3, 3] = handeye_translation.ravel()

print('Hand-eye calibration successful!\n\n')
print('Translation vector:\n')
print(handeye_translation, '\n\n')
print('Rotation vector:\n')
print(handeye_rotation, '\n')
print(compute_calibration_rmse(endeffector_poses, camera_poses, handeye_transform))

cv2.destroyAllWindows()

