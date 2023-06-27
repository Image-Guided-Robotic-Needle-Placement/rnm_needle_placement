#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from quintic_functions import calculateSmoothRobotTrajectory

current_joint_states = None
desired_joint_states = None

def joint_states_callback(msg):
    global current_joint_states
    current_joint_states = msg.position

def desired_goal_callback(msg):
    global desired_joint_states
    desired_joint_states = msg.position

def publish_trajectory():
    # Create ROS node
    rospy.init_node('trajectory_publisher', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('/joint_position_example_controller/joint_command', Float64MultiArray, queue_size=10)

    # Subscribe to joint states topic
    rospy.Subscriber('/joint_states_desired', JointState, joint_states_callback)

    # Subscribe to desired goal topic
    rospy.Subscriber('/desired_goal_states', JointState, desired_goal_callback)

    # Wait for the initial joint states and desired joint states to be received
    while current_joint_states is None or desired_joint_states is None:
        rospy.sleep(0.1)

    # Calculate trajectory
    trajectories = calculateSmoothRobotTrajectory(current_joint_states, desired_joint_states, True)
    print("Trajectory calculated.....publishing....")

    # Loop through the trajectory and publish each row
    rate = rospy.Rate(1000)  # Hz
    for trajectory_point in trajectories:
        # Create Float64MultiArray message
        msg = Float64MultiArray()
        msg.data = trajectory_point.tolist()

        # Publish the message
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':

    # Call the function to publish the trajectory
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
