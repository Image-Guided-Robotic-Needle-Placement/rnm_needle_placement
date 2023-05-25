#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import tf
import numpy as np
from numba import jit
from sympy import lambdify, symbols, Matrix, Abs, cos, sin, pi

q1, q2, q3, q4, q5, q6, q7 = symbols('theta1:8')
d, a, alpha, theta = symbols('d a alpha theta')

class InverseKinematics:
    def __init__(self):     
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.pose_callback)
        self.joint_angles_publisher = rospy.Publisher("/goal_states", JointState, queue_size = 10)
        self.r = rospy.Rate(1)
        self.joint_angles = Matrix([q1, q2, q3, q4, q5, q6, q7])
        #dh parameters
        self.alpha = np.array([0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2])
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0.107])
        self.jointstate = JointState()

    def pose_callback(self, msg):
        """Callback function for the subscriber.
           Gets the current joint angles and calculates FK and IK.
           publishes the joint angles to the /goal_states topic.
        """
        self.q = np.array(msg.position)
        self.DK = self.dh_calculation(self.q)    
        DK = self.DK[0:3, 0:4]
        self.A = DK.transpose().reshape(12, 1)
        self.J = self.A.jacobian(self.joint_angles)
        self.A_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), self.A, 'numpy'), nopython=True)       #issues in no_python argument
        self.J_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), self.J, 'numpy'), nopython=True)
        self.q = self.inverse_kinematics()

        #publishing the returned list/array as a JointState message
        self.jointstate.position = self.q.flatten()
        self.joint_angles_publisher.publish(self.jointstate)
        self.r.sleep()
        
    def dh_calculation(self, q):
        q = q.flatten() 
        q = [0,0,0,0,0,0,0]
        DK = np.identity(4)
        #general transformation matrix
        transform = Matrix([[cos(theta), -sin(theta), 0, a],
                            [sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                            [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                            [0, 0, 0, 1]])
        for i in range(len(q)):
            DK = DK @ transform.subs({theta: q[i], alpha: self.alpha[i], a: self.a[i], d: self.d[i]})
        return DK

    def incremental_ik(self, q_current, A_current, A_final, step=0.1, atol = 1e-6):
        """Gets the current joint angles, current pose and the final pose.
           Calculates the error and updates the joint angles until the error is less than the tolerance.
        """
        delta_A = (A_final - A_current)  #error
        while np.max(np.abs(delta_A)) > atol:
            J_q = self.J_lambdified(*(q_current.flatten()))
            J_q = J_q/np.linalg.norm(J_q)   #normalizing to avoid larger angles
            delta_q = np.linalg.pinv(J_q) @ (delta_A*step)
            q_current = q_current + delta_q
            A_current = self.A_lambdified(*(q_current.flatten()))
            delta_A = (A_final - A_current)
        return q_current, A_current
            
    def inverse_kinematics(self):
        #TODO: #define the q_init, A_init and A_final

        q_current = np.array([0, 0, 0, 0, 0, 0, 0]).reshape(7,1)
        A_current = self.A_lambdified(*(q_current.flatten()))
        q_rand = np.array([5, 5, 5, 5, 5, 5, 5]).reshape(7,1)
        A_final = self.A_lambdified(*(q_rand.flatten()))
        q, _ = self.incremental_ik(q_current, A_current, A_final)
        return q
    
if __name__ == "__main__":
    rospy.init_node("inverse_kinematics")
    ik = InverseKinematics()
    while not rospy.is_shutdown():
        rospy.spin()