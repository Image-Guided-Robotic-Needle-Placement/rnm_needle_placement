#!/usr/bin/env python3

"""
Author: David Sosa Gomez, Manav Thakkar and Omar Draidrya

This node is used to publish the initial position of the robot to the /goal_states.
"""

import rospy
from sensor_msgs.msg import JointState

def initial_position_publisher():
    # Initialize the node
    rospy.init_node('initial_position_node')

    pub = rospy.Publisher('/goal_states', JointState, queue_size=10)

    # Define joint names for Panda robot
    joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]

    # Define the joint angles
    joint_positions = [-0.06345210742427591, -0.40445922335440626, 0.005874457170912318, -1.2728393697738647, 0.03285966468819975, 0.8744066302879874, -0.0010685555618854413]

    # Create a JointState message
    msg = JointState()
    msg.name = joint_names
    msg.position = joint_positions

    rospy.sleep(1)

    # Publish the message once
    pub.publish(msg)

    rospy.loginfo("Published goal joint state to /goal_states")

if __name__ == '__main__':
    try:
        initial_position_publisher()
    except rospy.ROSInterruptException:
        pass
