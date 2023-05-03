#Performing forward knematics for franka panda robot (converting joint positions to a robot space in cartesian space)
# current joint positions can be accessed from the topic /joint_states

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf


class ForwardKinematics:
    def __init__(self):
        rospy.init_node("forward_kinematics", anonymous = True)
        self.jointstate_subscriber = rospy.Subscriber("/joint_states", JointState, self.jointstate_callback)
        self.alpha = np.array([0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2])
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0.107])
        self.r = rospy.Rate(10)

    def jointstate_callback(self, msg):
        self.q = np.array(msg.position, dtype = np.float64)
        #publishing the pose message we calculated using forward kinematics
        pose_in_cartesian = self.forward_kinematics()
        self.pose_pub = rospy.Publisher("/end_effector_pose", Pose, queue_size = 10)
        self.pose_pub.publish(pose_in_cartesian)
        self.r.sleep()

    def dh_between_frames(self, q, a, alpha, d):
        #general formula for transformation between two frames
        tf_matrix_between_frames = np.array([[np.cos(q), -np.sin(q), 0, a],
                                          [np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                                          [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
                                          [0, 0, 0, 1]])
        return tf_matrix_between_frames

    def forward_kinematics(self):
        #from base(0) to end effector(6)
        for i in range(0, 7):
            if i == 0:
                T_0_7 = self.dh_between_frames(self.q[i], self.a[i], self.alpha[i], self.d[i])
            else:
                T_0_7 = np.dot(T_0_7, self.dh_between_frames(self.q[i], self.a[i], self.alpha[i], self.d[i]))        
        #Converting tf_matrix to position and orientation to publish as a pose type msg 
        # https://answers.ros.org/question/379109/transformation-matrices-to-geometry_msgspose/
        q = tf.transformations.quaternion_from_matrix(T_0_7)
        pose = Pose()
        pose.position.x = T_0_7[0, 3]
        pose.position.y = T_0_7[1, 3]
        pose.position.z = T_0_7[2, 3]
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose

if __name__ == "__main__":
    fk = ForwardKinematics()
    while not rospy.is_shutdown():
        rospy.spin()

# pose.position.x = T_0_7[0, 3]
# pose.position.y = T_0_7[1, 3]
# pose.position.z = T_0_7[2, 3]
# pose.orientation.x = T_0_7[0, 0]
# pose.orientation.y = T_0_7[1, 0]
# pose.orientation.z = T_0_7[2, 0]
# pose.orientation.w = 1
#return pose

# T_0_7 = self.dh_between_frames(self.q[0], self.a[0], self.alpha[0], self.d[0])
# for i in range(1, 7):
#     T_0_7 = np.dot(T_0_7, self.dh_between_frames(self.q[i], self.a[i], self.alpha[i], self.d[i]))
# return T_0_7

    


