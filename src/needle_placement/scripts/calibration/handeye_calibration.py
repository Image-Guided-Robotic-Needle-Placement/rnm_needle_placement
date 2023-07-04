import numpy as np
import cv2

def read_endeffector_poses():
    poses = np.load('/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/scripts/calibration/endeffector_poses.npy')
    translations = []
    rotations = []
    for pose in poses:
        translations.append(np.array(pose[0:3,3]))
        rotations.append(np.array(pose[0:3,0:3]))
    # with open('endeffector_poses.txt', 'r') as file:
    #     content = file.read()

    # pose_strings = content.split('\n\n\n')
    # poses = []

    # for pose_string in pose_strings:
    #     lines = pose_string.split('\n')
    #     current_pose = {}

    #     for line in lines:
    #         if line.startswith('position'):
    #             current_pose['translation'] = np.array([float(lines[1].split(':')[1].strip()) * 1000,
    #                                                     float(lines[2].split(':')[1].strip()) * 1000,
    #                                                     float(lines[3].split(':')[1].strip()) * 1000])
    #         elif line.startswith('orientation'):
    #             current_pose['rotation'] = np.array([float(lines[5].split(':')[1].strip()),
    #                                                  float(lines[6].split(':')[1].strip()),
    #                                                  float(lines[7].split(':')[1].strip()),
    #                                                  float(lines[8].split(':')[1].strip())])
    #             current_pose['rotation'] /= np.linalg.norm(current_pose['rotation'])
    #             poses.append(current_pose)

    # translations = []
    # rotations = []
    # for i, pose in enumerate(poses):
    #     translation = pose['translation']
    #     rotation_quat = pose['rotation']

    #     # Compute rotation matrix from quaternion
    #     x, y, z, w = rotation_quat
    #     rotation_matrix = np.array([[1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y],
    #                                 [2 * x * y + 2 * w * z, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * w * x],
    #                                 [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, 1 - 2 * x ** 2 - 2 * y ** 2]])

    #     translations.append(translation)
    #     rotations.append(rotation_matrix)

    # translations = np.array(translations)
    # rotations = np.array(rotations)
    return translations, rotations

def read_board_poses():
    # with open('/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/scripts/calibration/board_poses_final.txt', 'r') as file:
    #     content = file.read()

    # pose_strings = content.strip().split('\n\n')

    # poses = []
    # translations = []
    # rotations = []

    # for pose_string in pose_strings:
    #     lines = pose_string.split('\n')

    #     if lines[0] != '':
    #         # Extract translation vector
    #         translation = np.array([float(num) for num in lines[0].strip().strip('[]').split()])
    #         translations.append(translation)
    #         # Extract rotation matrix
    #         rotation_matrix = np.array([[float(num) for num in row.strip().strip('[]').split()] for row in lines[1:]])
    #         rotations.append(rotation_matrix)
    #         poses.append({'translation': translation, 'rotation': rotation_matrix})
    #     else:
    #                     # Extract translation vector
    #         translation = np.array([float(num) for num in lines[1].strip().strip('[]').split()])
    #         translations.append(translation)
    #         # Extract rotation matrix
    #         rotation_matrix = np.array([[float(num) for num in row.strip().strip('[]').split()] for row in lines[2:]])
    #         rotations.append(rotation_matrix)
    #         poses.append({'translation': translation, 'rotation': rotation_matrix})
    poses = np.load('/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/scripts/calibration/board_poses.npy')
    translations = []
    rotations = []
    for pose in poses:
        translations.append(np.array(pose[0:3,3]))
        rotations.append(np.array(pose[0:3,0:3]))

    return translations, rotations


# Process each pose
endeffector_translations, endeffector_rotations = read_endeffector_poses()
camera_translations, camera_rotations = read_board_poses()

handeye_rotation, handeye_translation = cv2.calibrateHandEye(endeffector_rotations, endeffector_translations, camera_rotations, camera_translations, cv2.CALIB_HAND_EYE_HORAUD)

print('Hand-eye calibration successful!\n\n')
print('Translation vector:\n')
print(handeye_translation, '\n\n')
print('Rotation vector:\n')
print(handeye_rotation, '\n')
cv2.destroyAllWindows()

