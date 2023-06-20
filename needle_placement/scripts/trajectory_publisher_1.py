#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from rnm_trajectory_function import trajectory_generation

current_joint_states = None
desired_joint_states = None

def joint_states_callback(msg):
    global current_joint_states
    current_joint_states = msg.position

def desired_goal_callback(msg):
    global desired_joint_states
    desired_joint_states = msg.position

def publish_trajectory():
    # Create publisher
    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)

    # Wait for the initial joint states and desired joint states to be received
    while current_joint_states is None or desired_joint_states is None:
        rospy.sleep(0.01)

    while not rospy.is_shutdown():
        # Wait for new desired joint states
        while desired_joint_states is None:
            rospy.sleep(0.01)

        # Store the current desired joint states to compare later
        current_desired_joint_states = desired_joint_states

        # Calculate trajectory
        trajectories, duration, num_points = trajectory_generation(current_joint_states, desired_joint_states, duration, num_points)

        # Loop through the trajectory and publish each row
        #rate = rospy.Rate(1.0 / (duration / num_points))  # Publish frequency based on duration and num_points
        rate = rospy.Rate(1000)
        for trajectory_point in trajectories:
            # Check if desired joint states have changed
            if desired_joint_states != current_desired_joint_states:
                break  # Break the loop if new desired joint states received

            # Create Float64MultiArray message
            msg = Float64MultiArray()
            msg.data = trajectory_point.tolist()

            # Publish the message
            pub.publish(msg)

            rate.sleep()
        
        rospy.loginfo("Goal Reached!")
        

def main():
    # Create ROS node
    rospy.init_node('trajectory_publisher', anonymous=True)

    # Subscribe to joint states topic
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # Subscribe to desired goal topic
    rospy.Subscriber('/desired_goal_states', JointState, desired_goal_callback)

    #if len(sys.argv) < 3:
    #    print("Usage: python trajectory_publisher.py <duration> <num_points>")
    #    sys.exit(1)

    # Extract duration and num_points from command-line arguments
    #duration = float(sys.argv[1])
    #num_points = int(sys.argv[2])

    # Call the function to publish the trajectory
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
