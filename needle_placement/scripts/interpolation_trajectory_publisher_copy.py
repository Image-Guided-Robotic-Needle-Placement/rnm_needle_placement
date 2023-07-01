#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from straight_line_function_copy import interpolate

def publish_trajectory():
    # Create ROS node
    rospy.init_node('trajectory_publisher', anonymous=True)

    # Create publisher
    pub = rospy.Publisher('/linear_transformations', Float64MultiArray, queue_size=10)

    # Define the matrices
    A_entry = np.array([[ 0.95663454,  0.28769805,  0.04560904, 0.283],
                            [ 0.23035644, -0.84302106,  0.486057, -0.208],
                            [ 0.17828703, -0.45447258, -0.87273616, 0.366],
                            [ 0.0, 0.0, 0.0, 1.0]])

    A_ball = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                       [0.19840159, -0.83371212, 0.51532603, -0.139],
                       [0.2377198, -0.46914648, -0.85052388, 0.266],
                       [0.0, 0.0, 0.0, 1.0]])

    # Calculate trajectory
    t = np.linspace(0, 1, 10)
    trajectories = interpolate(A_entry, A_ball, t)
    print("Trajectory calculated.....publishing....")

    # Loop through the trajectory and publish each row
    rate = rospy.Rate(1000)  # Hz
    for trajectory_point in trajectories:
        # Create Float64MultiArray message
        msg = Float64MultiArray()
        msg.data = trajectory_point.flatten().tolist()

        # Publish the message
        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':

    # Call the function to publish the trajectory
    try:
        publish_trajectory()
    except rospy.ROSInterruptException:
        pass
