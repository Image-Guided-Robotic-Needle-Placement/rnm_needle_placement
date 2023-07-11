#!/usr/bin/env python3

"""
Author: David Sosa Gomez and Manav Thakkar

This node takes /A_entry , uses the inverse kinematics function to calculate the joint angles and publishes them to /goal_states.
With this node the robot moves to the entry point.
"""

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from inverse_kinematics_function import inverse_kinematics

# This function will be called every time a new message is received on the /joint_states topic
def joint_states_callback(msg):
    global current_joint_position
    current_joint_position = msg.position
    
# This function will be called every time a new message is received on the /A_entry topic
def A_entry_callback(msg):
    global A_entry
    A_entry = np.array(msg.data).T.reshape(-1, 1)
    print("A_entry received", A_entry)

    
if __name__ == "__main__":
    rospy.init_node('entry_point_pubsub')
    pub = rospy.Publisher('/goal_states', JointState, queue_size=10)

    # Subscribe to the /joint_states topic
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # Subscribe to the /A_entry topic
    rospy.Subscriber('/A_entry', Float64MultiArray, A_entry_callback)

    # Initialize current_joint_position
    current_joint_position = None
    A_entry = None

    # Wait until current_joint_position and A_entry are initialized by the first message on their respective topics
    while current_joint_position is None or A_entry is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

    rate = rospy.Rate(10)

    print("calculating inverse kinematics...")
    final_joint_angles = inverse_kinematics(current_joint_position, A_entry)
    print("done calculating inverse kinematics")

    # Update the current joint position with the newly calculated final joint angles
    current_joint_position = final_joint_angles.tolist()

    joint_angles_msg = JointState()
    joint_angles_msg.position = final_joint_angles

    pub.publish(joint_angles_msg)
    print("published joint angles", joint_angles_msg.position)
    rate.sleep()
    
    # Shutdown the node
    rospy.signal_shutdown('Finished publishing all angles')  
