#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

def interpolate(A_entry, A_ball, t):

    return (1.0 - t) * A_entry + t * A_ball

if __name__ == "__main__":
    rospy.init_node('interpolation_publisher')
    pub = rospy.Publisher('/linear_transformations', Float32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    A_entry = np.array([[0.95663454, 0.28769805, 0.04560904, 0.283],
                        [0.23035644, -0.84302106, 0.486057, -0.208],
                        [0.17828703, -0.45447258, -0.87273616, 0.366],
                        [0.0, 0.0, 0.0, 1.0]])

    A_ball = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                       [0.19840159, -0.83371212, 0.51532603, -0.139],
                       [0.2377198, -0.46914648, -0.85052388, 0.266],
                       [0.0, 0.0, 0.0, 1.0]])
    
    while not rospy.is_shutdown():
        for t in np.linspace(0, 1, 100):
            interpolated_pose = interpolate(A_entry, A_ball, t)
            transformation_msg = Float32MultiArray()
            transformation_msg.data = interpolated_pose.flatten()

            pub.publish(transformation_msg)
            rate.sleep()
