#!/usr/bin/env python3

import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, Bool
from quintic_functions import calculateSmoothRobotTrajectory
from inverse_kinematics_function import inverse_kinematics
from sensor_msgs.msg import JointState

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
        
    def interpolate(self, A_entry, A_ball, t):
        return (1.0 - t) * A_entry + t * A_ball

    def run(self):
        while not rospy.is_shutdown() and (self.current_joint_position is None or self.A_entry is None or self.A_ball is None or self.reached is False):
            rospy.sleep(0.1)

        for t in np.linspace(0, 1, 10):
            interpolated_pose = self.interpolate(self.A_entry, self.A_ball, t)
            print("Calculating inverse kinematics...")
            final_joint_angles = inverse_kinematics(self.current_joint_position, interpolated_pose)
            self.current_joint_position = final_joint_angles

            joint_angles_msg = JointState()
            joint_angles_msg.position = final_joint_angles

            self.pub.publish(joint_angles_msg)
            self.rate.sleep()


if __name__ == "__main__":
    interpolation_publisher = InterpolationPublisher()
    interpolation_publisher.run()
