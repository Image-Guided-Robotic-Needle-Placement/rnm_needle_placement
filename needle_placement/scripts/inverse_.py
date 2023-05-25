#!/usr/bin/env python3
import rospy
import sys
import numpy as np
from sensor_msgs.msg import JointState
import tf


class Inverse_:
    def __init__(self):
        self.jointstate_subscriber = rospy.Subscriber("/joint_states", JointState, self.jointstate_callback)
        self.alpha = np.array([0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2])
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0.107])
        self.r = rospy.Rate(1)


    def jointstate_callback(self, msg):
        #self.q = np.array(msg.position, dtype = np.float64)

        #as of now getting zeros (for checking)
        self.q = np.zeros(7)
        #publishing the pose message we calculated using forward kinematics
        current_tf_matrix = self.forward_kinematics_solver(self.q)
        #converting the tf matrix into [x, y, z, qx, qy, qz, qw] and passing it into inverse kinematics solver
        q = tf.transformations.quaternion_from_matrix(current_tf_matrix)
        x = current_tf_matrix[0, 3]
        y = current_tf_matrix[1, 3]
        z = current_tf_matrix[2, 3]
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]
        self.current_pose = np.array([x, y, z, qx, qy, qz, qw])
        self.desired_pose = np.array([0.5, 0.5, 0.5, 0, 0, 0, 1])
        self.desrired_joints = self.inverse_kinematics_solver(self.desired_pose, self.current_pose)

        

    def dh_between_frames(self, q, a, alpha, d):
        #general formula for transformation between two frames
        tf_matrix_between_frames = np.asarray([[np.cos(q), -np.sin(q), 0, a],
                                            [np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                                            [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
                                            [0, 0, 0, 1]])
        return tf_matrix_between_frames

    def forward_kinematics_solver(self, q):
        
        for i in range(len(q)):
            if i == 0:
                T_0_7 = self.dh_between_frames(q[i], self.a[i], self.alpha[i], self.d[i])
            else:
                T_0_7 = np.dot(T_0_7, self.dh_between_frames(q[i], self.a[i], self.alpha[i], self.d[i]))
        return T_0_7
    
    def jacobian_solver(self,current_joints):
        """standard way of calculating jacobian
           gets the current joints, calculates the fk. From the fk, we will find the difference and calculate the joint velocities
        """
        jacobian = np.zeros((6,7))
        dh_fk = self.forward_kinematics_solver(current_joints)
        for i in range(7):
            orientation = np.array([dh_fk[0, 2], dh_fk[1, 2], dh_fk[2, 2]]) 
            position = np.array([dh_fk[0, 3], dh_fk[1, 3], dh_fk[2, 3]])   #EE position wrt base frame  
            position_vector = np.array([self.a[i], self.d[i]*np.sin(self.alpha[i]), self.d[i]*np.cos(self.alpha[i])]) #position of joint i in base frame
            jacobian[0:3, i] = np.cross(orientation, position - position_vector)
            jacobian[3:6, i] = orientation
        return jacobian
    
    def inverse_kinematics_solver(self, desired_pose, current_pose):
        joints = current_pose
        error = desired_pose - current_pose
        while np.linalg.norm(error) > 1e-6:
            jacobian = self.jacobian_solver(current_pose)
            print("shape of error: ", error.shape)
            joint_error = np.dot(np.linalg.pinv(jacobian), error)
            print("shape of joint_error: ", joint_error.shape)
            joints = joints + joint_error         
            current_pose = self.forward_kinematics_solver(joints)
            error = desired_pose - current_pose
            self.r.sleep()
        return current_pose

if __name__ == "__main__":
    rospy.init_node("inverse_kinematics")
    inverse_kinematics = Inverse_()
    rospy.spin()

