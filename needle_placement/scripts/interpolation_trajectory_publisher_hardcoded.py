#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from inverse_kinematics_function import inverse_kinematics
import time

def interpolate(A_entry, A_ball, t):
    return (1.0 - t) * A_entry + t * A_ball

# This function will be called every time a new message is received on the /joint_states topic
def joint_states_callback(msg):
    global current_joint_position
    current_joint_position = msg.position

if __name__ == "__main__":
    rospy.init_node('interpolation_publisher')
    pub = rospy.Publisher('/goal_states', JointState, queue_size=1)

    # Subscribe to the /joint_states topic
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # Initialize current_joint_position
    current_joint_position = None

    # Wait until current_joint_position is initialized by the first message on the /joint_states topic
    while current_joint_position is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    rate = rospy.Rate(0.5)

    A_entry = np.array([[0.95663454, 0.28769805, 0.04560904, 0.283],
                        [0.23035644, -0.84302106, 0.486057, -0.208],
                        [0.17828703, -0.45447258, -0.87273616, 0.366]]).T.reshape(-1, 1)

    A_ball = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                       [0.19840159, -0.83371212, 0.51532603, -0.139],
                       [0.2377198, -0.46914648, -0.85052388, 0.266]]).T.reshape(-1, 1)

    for t in np.linspace(0, 1, 5):
        interpolated_pose = interpolate(A_entry, A_ball, t)
        print("calculating inverse kinematics...")
        final_joint_angles = inverse_kinematics(current_joint_position, interpolated_pose)
        print("done calculating inverse kinematics")
        current_joint_position = final_joint_angles.tolist()

        joint_angles_msg = JointState()
        joint_angles_msg.position = final_joint_angles

        time.sleep(10)
        pub.publish(joint_angles_msg)
        rate.sleep()

    rospy.signal_shutdown('Finished publishing all angles')  # Shutdown the node
