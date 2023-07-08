import copy

import numpy as np

def get_rotation_matrix(a, b):
    return


ball_point_scan = np.array([-0.0038827, -0.02626073, 0.03048087])
entry_point_scan = np.array([-0.04147948, -0.17644563, 0.146996])

# Hard-coded values obtained from select_points.py
ball_point_scan_homogeneous = np.array([ball_point_scan[0], ball_point_scan[1], ball_point_scan[2], 1])  # is the scaling (* 0.001) correct?
entry_point_scan_homogeneous = np.array([entry_point_scan[0], entry_point_scan[1], entry_point_scan[2], 1])

# We don't neeed it, because the registration already brings the scan in the world coordinate system
# handeye_transformation = np.array([[0.67808914, -0.04218229,   0.7337682, 0.05999226],
#                                    [-0.7349356, -0.04984826,  0.67630231, 0.01110063],
#                                    [0.00804909, -0.99786562, -0.06480283, 0.04734731],
#                                    [0, 0, 0, 1]])

registration_transformation = np.array([[-0.95390457, 0.24106763, -0.17875251, 0.35151915],
                                        [0.2094595, 0.96134724, 0.17871261, -0.12679727],
                                        [0.21492506, 0.13303337, -0.96752744, -0.04682151],
                                        [0, 0, 0, 1]])

registration_transformation_inverse = np.linalg.inv(registration_transformation)

ee_to_needle = np.eye(4)
ee_to_needle[0:3, 3] = [-0.00156939339, 0.00076773158, 0.19702979428]
needle_to_ee = np.linalg.inv(ee_to_needle)

ball_point_global = np.matmul(registration_transformation_inverse, ball_point_scan_homogeneous)
entry_point_global = np.matmul(registration_transformation_inverse, entry_point_scan_homogeneous)

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

needle_pose_entry[0:3, 0:3] = np.array([np.transpose(direction_vector_X), np.transpose(direction_vector_Y), np.transpose(direction_vector_Z)])
needle_pose_entry[0:3, 3] = entry_point_global[0:3]

needle_pose_ball[0:3, 0:3] = np.array([np.transpose(direction_vector_X), np.transpose(direction_vector_Y), np.transpose(direction_vector_Z)])
needle_pose_ball[0:3, 3] = ball_point_global[0:3]

ee_pose_entry = np.matmul(needle_to_ee, needle_pose_entry)
ee_pose_ball = np.matmul(needle_to_ee, needle_pose_ball)

print("EE entry pose:", ee_pose_entry)
print("EE ball pose:", ee_pose_ball)

