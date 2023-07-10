import copy
import numpy as np


# Hard-coded values of ball- and entry-point in the provided scan obtained from select_points.py
# CS: local scan CS
ball_point_scan_homogeneous = np.array([-0.00445699, -0.02528321, 0.02605761, 1])
entry_point_scan_homogeneous = np.array([-0.03418263, -0.18129, 0.34387507, 1])

''' #  We don't neeed it, because the registration already brings the scan in the world coordinate system
handeye_transformation = np.array([[0.67808914, -0.04218229,   0.7337682, 0.05999226],
                                   [-0.7349356, -0.04984826,  0.67630231, 0.01110063],
                                   [0.00804909, -0.99786562, -0.06480283, 0.04734731],
                                   [0, 0, 0, 1]])'''

# Registration provided model -> our model
registration_transformation = np.array([[0.94899234, -0.06606545, -0.30830002, 0.34693045],
                                        [0.12865134, 0.97383691, 0.18732458, -0.09952967],
                                        [0.28785826, -0.2174328,  0.93266318, 0.05944911],
                                        [0, 0, 0, 1]])


# Registration provided model -> our model
registration_transformation_inverse = np.linalg.inv(registration_transformation)

#  Transformation end-effector -> need-tip
ee_to_needle = np.eye(4)
ee_to_needle[0:3, 3] = [-0.00156939339, 0.00076773158, 0.19702979428]

# Transformation needle-tip -> end-effector
needle_to_ee = np.linalg.inv(ee_to_needle)

# Ball- and entry-points in the global coordinate system
# CS: global
ball_point_global = np.matmul(registration_transformation, ball_point_scan_homogeneous)
entry_point_global = np.matmul(registration_transformation, entry_point_scan_homogeneous)

# entry_point_global = np.matmul(needle_to_ee, entry_point_global)
# ball_point_global = np.matmul(needle_to_ee, ball_point_global)

# Align the needle with the entry-ball axis
# Direction vector Z
direction_vector_Z = ball_point_global - entry_point_global
direction_vector_Z = direction_vector_Z[0:3]
direction_vector_Z = direction_vector_Z / np.linalg.norm(direction_vector_Z)

# Direction vector Y
direction_vector_Y = np.array([1, 1, -(direction_vector_Z[0] + direction_vector_Z[1])/direction_vector_Z[2]])
direction_vector_Y = direction_vector_Y / np.linalg.norm(direction_vector_Y)

# Direction vector X
direction_vector_X = np.cross(direction_vector_Y, direction_vector_Z)

needle_pose_ball = np.eye(4)
needle_pose_entry = np.eye(4)

needle_pose_entry[0:3, 0:3] = np.array([direction_vector_X, direction_vector_Y, direction_vector_Z])
needle_pose_entry[0:3, 0:3] = np.transpose(needle_pose_entry[0:3, 0:3])
needle_pose_entry[0:3, 3] = entry_point_global[0:3]

needle_pose_ball[0:3, 0:3] = np.array([direction_vector_X, direction_vector_Y, direction_vector_Z])
needle_pose_ball[0:3, 0:3] = np.transpose(needle_pose_ball[0:3, 0:3])
needle_pose_ball[0:3, 3] = ball_point_global[0:3]

# Compute the end-effector poses
ee_pose_entry = np.matmul(ee_to_needle, needle_pose_entry)
ee_pose_ball = np.matmul(ee_to_needle, needle_pose_ball)

print("EE entry pose:", ee_pose_entry)
print("EE ball pose:", ee_pose_ball)

