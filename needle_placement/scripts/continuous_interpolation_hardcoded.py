#!/usr/bin/env python3

import numpy as np
import rospy
from quintic_functions import calculateSmoothRobotTrajectory
from inverse_kinematics_function import inverse_kinematics
from sensor_msgs.msg import JointState

class InterpolationPublisher:
    def __init__(self):
        rospy.init_node('interpolation_publisher')
        self.pub = rospy.Publisher('/interpolated_angles', JointState, queue_size=1)
        rospy.Subscriber('/joint_states', JointState, self.joint_position_callback)

        self.A_entry = np.array([[0.95663454, 0.28769805, 0.04560904, 0.283],
                                 [0.23035644, -0.84302106, 0.486057, -0.208],
                                 [0.17828703, -0.45447258, -0.87273616, 0.366]]).T.reshape(-1, 1)

        self.A_ball = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                                [0.19840159, -0.83371212, 0.51532603, -0.139],
                                [0.2377198, -0.46914648, -0.85052388, 0.266]]).T.reshape(-1, 1)
        self.current_joint_position = None

        self.rate = rospy.Rate(1)

    def joint_position_callback(self, msg):
        self.current_joint_position = msg.position
        
    def interpolate(self, A_entry, A_ball, t):
        return (1.0 - t) * A_entry + t * A_ball

    def run(self):
        while not rospy.is_shutdown() and self.current_joint_position is None:
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
