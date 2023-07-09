import copy
import numpy as np


# Hard-coded values of ball- and entry-point in the local coordinate system of the provided scan obtained from select_points.py
ball_point_scan_homogeneous = np.array([-0.0038827, -0.02626073, 0.03048087, 1])
entry_point_scan_homogeneous = np.array([-0.04147948, -0.17644563, 0.146996, 1])

''' #  We don't neeed it, because the registration already brings the scan in the world coordinate system
handeye_transformation = np.array([[0.67808914, -0.04218229,   0.7337682, 0.05999226],
                                   [-0.7349356, -0.04984826,  0.67630231, 0.01110063],
                                   [0.00804909, -0.99786562, -0.06480283, 0.04734731],
                                   [0, 0, 0, 1]])'''

# Registration from our model to the provided model
registration_transformation = np.array([[-0.95390457, 0.24106763, -0.17875251, 0.35151915],
                                        [0.2094595, 0.96134724, 0.17871261, -0.12679727],
                                        [0.21492506, 0.13303337, -0.96752744, -0.04682151],
                                        [0, 0, 0, 1]])

registration_transformation_inverse = np.linalg.inv(registration_transformation)

#  Transformation end-effector -> need-tip
ee_to_needle = np.eye(4)
ee_to_needle[0:3, 3] = [-0.00156939339, 0.00076773158, 0.19702979428]
needle_to_ee = np.linalg.inv(ee_to_needle)

# Ball- and entry-points in the global coordinate system
ball_point_global = np.matmul(registration_transformation_inverse, ball_point_scan_homogeneous)
entry_point_global = np.matmul(registration_transformation_inverse, entry_point_scan_homogeneous)

# Align the needle with the entry-ball axis
# Direction vector Z
direction_vector_Z = ball_point_global - entry_point_global
direction_vector_Z = direction_vector_Z[0:3]
direction_vector_Z = direction_vector_Z / np.linalg.norm(direction_vector_Z)

# Direction vector Y
direction_vector_Y = copy.deepcopy(direction_vector_Z)
direction_vector_Y[2] = -direction_vector_Y[2]

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

