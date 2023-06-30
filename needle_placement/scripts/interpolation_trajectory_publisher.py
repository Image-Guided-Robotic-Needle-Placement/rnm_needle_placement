#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import numpy as np
from straight_line_function import PoseInterpolator

A_entry = None
A_ball = None

def A_entry_callback(msg):
    global A_entry
    # Assuming you get pose as a 1D array of 16 elements, you need to reshape it
    A_entry = np.array(msg.position).reshape((4, 4))

def A_ball_callback(msg):
    global A_ball
    # Assuming you get pose as a 1D array of 16 elements, you need to reshape it
    A_ball = np.array(msg.position).reshape((4, 4))

def publish_trajectory():
    # Create ROS node
    rospy.init_node('trajectory_publisher', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)

    # Subscribe to A_entry and A_ball topics
    rospy.Subscriber('/A_entry', Float64MultiArray, A_entry_callback)
    rospy.Subscriber('/A_ball', Float64MultiArray, A_ball_callback)

    # Wait for the initial joint states and desired joint states to be received
    while A_entry is None or A_ball is None:
        rospy.sleep(0.1)

    # Initialize the interpolator
    interpolator = PoseInterpolator(A_entry, A_ball)

    # Loop through the interpolation parameter t and publish each interpolated pose
    rate = rospy.Rate(1000)  # Hz
    for t in np.linspace(0, 1, 100):
        # Calculate interpolated pose
        interpolated_pose = interpolator.interpolate(t)

        # Create Float64MultiArray message
        msg = Float64MultiArray()
        # Flatten the 2D array to 1D before sending
        msg.data = interpolated_pose.flatten().tolist()

        # Publish the message
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    # Call the function to publish the trajectory
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
