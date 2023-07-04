import numpy as np
from quintic_functions import calculateSmoothRobotTrajectory
from inverse_kinematics import inverse_kinematics

def interpolate(A_entry, A_ball, t):
    return (1.0 - t) * A_entry + t * A_ball

A_entry = np.array([[0.95663454, 0.28769805, 0.04560904, 0.283],
                    [0.23035644, -0.84302106, 0.486057, -0.208],
                    [0.17828703, -0.45447258, -0.87273616, 0.366]]).T.reshape(-1, 1)

A_ball = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                    [0.19840159, -0.83371212, 0.51532603, -0.139],
                    [0.2377198, -0.46914648, -0.85052388, 0.266]]).T.reshape(-1, 1)
for t in np.linspace(0, 1, 5):
        current_joint_position = [0.33126478636661577, -0.8257679608411956, -1.0212689482370507, -2.6604452104735272, -0.21414480587762708, 1.936938619928904, -0.6195934342626068]
        interpolated_pose = interpolate(A_entry, A_ball, t)
        print("calculating inverse kinematics...")
        final_joint_angles = inverse_kinematics(current_joint_position, interpolated_pose)