#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from quintic_trajectory_old import calculate_all_joint_trajectories
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym
import roboticstoolbox as rtb

current_joint_states = None

def joint_states_callback(msg):
    global current_joint_states
    current_joint_states = msg.position

def publish_trajectory(duration, num_points, desired_joint_states):
    # Create publisher
    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)

    # Wait for the initial joint states to be received
    while current_joint_states is None:
        rospy.sleep(0.01)

    while not rospy.is_shutdown():
        # Calculate trajectory
        trajectories = calculate_all_joint_trajectories(current_joint_states, desired_joint_states, duration, num_points)

        # Loop through the trajectory and publish each row
        rate = rospy.Rate(1.0 / (duration / num_points))  # Publish frequency based on duration and num_points
        for trajectory_point in trajectories:
            # Create Float64MultiArray message
            msg = Float64MultiArray()
            msg.data = trajectory_point.tolist()

            # Publish the message
            pub.publish(msg)

            rate.sleep()
        
        rospy.loginfo("Goal Reached!")
        break  # Stop the loop after the trajectory is published once

def main():
    # Create ROS node
    rospy.init_node('trajectory_publisher', anonymous=True)

    # Subscribe to joint states topic
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    if len(sys.argv) < 5:
        print("Usage: python trajectory_publisher.py <duration> <num_points> <x> <y> <z>")
        sys.exit(1)

    # Extract duration, num_points, and desired_position from command-line arguments
    duration = float(sys.argv[1])
    num_points = int(sys.argv[2])
    desired_position = [float(arg) for arg in sys.argv[3:6]]

    # Load the panda robot model
    panda = rtb.models.URDF.Panda()

    # Get point coordinates from command-line arguments
    point = SE3(desired_position[0] , desired_position[1] , desired_position[2])

    # Calculate inverse kinematics
    desired_joint_states = panda.ikine_LM(point).q

    # Call the function to publish the trajectory
    try:
        publish_trajectory(duration, num_points, desired_joint_states)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
