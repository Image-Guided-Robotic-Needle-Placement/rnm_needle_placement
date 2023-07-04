#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import numpy as np
from inverse_kinematics_function import inverse_kinematics

# Initialize the ROS node
rospy.init_node('entry_point_publisher', anonymous=True)

# Define a callback function for the subscriber
def callback(data):
    current_joint_position = data.position 
    A_entry = np.array([[0.95663454, 0.28769805, 0.04560904, 0.283],
                        [0.23035644, -0.84302106, 0.486057, -0.208],
                        [0.17828703, -0.45447258, -0.87273616, 0.366]]).T.reshape(-1, 1)

    angles = inverse_kinematics(current_joint_position, A_entry)

    print(angles)

# Create the subscriber, subscribing to the joint_states topic
sub = rospy.Subscriber("/joint_states", JointState, callback)

# Keep the node alive
rospy.spin()
