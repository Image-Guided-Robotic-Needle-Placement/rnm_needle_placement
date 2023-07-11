#!/usr/bin/env python3

"""
Author: David Sosa Gomez and Manav Thakkar

This node takes /A_entry and /A_ball, interpolates t points between them and calculates joint angles for each point using 
inverse kinematics and publishes them to /interpolated_angles.
"""

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Bool
from inverse_kinematics_function import inverse_kinematics
from sensor_msgs.msg import JointState
import time

class InterpolationPublisher:
    def __init__(self):
        rospy.init_node('interpolation_publisher')
        self.pub = rospy.Publisher('/interpolated_angles', JointState, queue_size=1)
        
        rospy.Subscriber('/joint_states', JointState, self.joint_position_callback)

        # Subscribe to A_entry and A_ball topics
        rospy.Subscriber('/A_entry', Float64MultiArray, self.entry_callback)
        rospy.Subscriber('/A_ball', Float64MultiArray, self.ball_callback)
        rospy.Subscriber('/reached', Bool, self.reached_callback)
        
        self.A_entry = None
        self.A_ball = None
        self.current_joint_position = None
        self.reached = False

        self.rate = rospy.Rate(1)

    def joint_position_callback(self, msg):
        self.current_joint_position = msg.position

    # New callbacks for A_entry and A_ball topics
    def entry_callback(self, msg):
        self.A_entry = np.array(msg.data).T.reshape(-1, 1)

    def ball_callback(self, msg):
        self.A_ball = np.array(msg.data).T.reshape(-1, 1)

    def reached_callback(self, msg):
        self.reached = msg.data

    # interpolation function
    def interpolate(self, A_entry, A_ball, t):
        # interpolate between A_entry and A_ball in t steps
        return (1.0 - t) * A_entry + t * A_ball

    def run(self):
        # wait until we get A_entry A_ball current_joint_position and we have reached the entry point
        while not rospy.is_shutdown() and (self.current_joint_position is None or self.A_entry is None or self.A_ball is None or self.reached is False):
            rospy.sleep(0.1)

        # when we reached the entry point, wait 1 second to ensure that every joint is not moving anymore
        time.sleep(1)

        # perform interpolation for 10 steps
        for t in np.linspace(0, 1, 10):
            # compute the interpolated pose
            interpolated_pose = self.interpolate(self.A_entry, self.A_ball, t)
            print("Calculating inverse kinematics...")
            # calculate the final joint angles using the inverse kinematics function
            final_joint_angles = inverse_kinematics(self.current_joint_position, interpolated_pose)
            self.current_joint_position = final_joint_angles

            joint_angles_msg = JointState()
            joint_angles_msg.position = final_joint_angles

            # publish the joint angles
            self.pub.publish(joint_angles_msg)
            self.rate.sleep()


if __name__ == "__main__":
    interpolation_publisher = InterpolationPublisher()
    interpolation_publisher.run()
