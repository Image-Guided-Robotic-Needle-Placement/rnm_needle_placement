#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

def initial_position_publisher():
    # Initialize the node
    rospy.init_node('initial_position_node')

    # Define the publisher, here it publishes to the /goal_states topic
    # and the message type is JointState
    pub = rospy.Publisher('/goal_states', JointState, queue_size=10)

    # Define the joint angles
    initial_joint_angles = [-0.06345210742427591, -0.40445922335440626, 0.005874457170912318, -1.2728393697738647, 0.03285966468819975, 0.8744066302879874, -0.0010685555618854413]

    # Initialize the JointState message
    joint_state_msg = JointState()

    # Populate the JointState message
    # In this case, I assume the robot has 7 joints named 'joint1' to 'joint7'.
    joint_state_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    joint_state_msg.position = initial_joint_angles

    # Sleep for a bit to make sure the publisher is connected
    rospy.sleep()

    # Publish the joint angles
    pub.publish(joint_state_msg)
    
    # Print the message
    rospy.loginfo("Published joint goal states.")

    if __name__ == '__main__':
        try:
            initial_position_publisher()
        except rospy.ROSInterruptException:
            pass

