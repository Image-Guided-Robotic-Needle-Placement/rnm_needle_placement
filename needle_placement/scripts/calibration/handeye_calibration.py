import numpy as np
import cv2


def read_poses_from_npy(path):
    poses = np.load(path)
    translations = []
    rotations = []

    for pose in poses:
        translations.append(np.array(pose[0:3, 3]))
        rotations.append(np.array(pose[0:3, 0:3]))

    return translations, rotations


endeffector_poses_path = 'D:/xzFACULTATE/SoSe23/rnm/rnm_needle_placement-lab-final-fr/rnm_needle_placement-lab-final-fr/src/needle_placement/lab/endeffector_poses.npy'
board_poses_path = 'D:/xzFACULTATE/SoSe23/rnm/rnm_needle_placement-lab-final-fr/rnm_needle_placement-lab-final-fr/src/needle_placement/lab/board_poses.npy'

# Load poses from files
endeffector_translations, endeffector_rotations = read_poses_from_npy(endeffector_poses_path)
camera_translations, camera_rotations = read_poses_from_npy(board_poses_path)

# Do the hand-eye calibration which finds the transformation camera -> end-effector
handeye_rotation, handeye_translation = cv2.calibrateHandEye(endeffector_rotations, endeffector_translations, camera_rotations, camera_translations)

print('Hand-eye calibration successful!\n\n')
print('Translation vector:\n')
print(handeye_translation, '\n\n')
print('Rotation vector:\n')
print(handeye_rotation, '\n')
cv2.destroyAllWindows()

