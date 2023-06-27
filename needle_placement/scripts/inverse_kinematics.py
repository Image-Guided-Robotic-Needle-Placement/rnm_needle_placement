#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import tf
import numpy as np
from numba import jit
from sympy import lambdify, symbols, Matrix, cos, sin, pi, eye
q1, q2, q3, q4, q5, q6, q7 = symbols('theta1, theta2, theta3, theta4, theta5, theta6, theta7')
d, a, alpha, theta = symbols('d a alpha theta')

@jit(nopython=True)
def incremental_ik(q, A_current, A_final, A_lambdified, J_lambdified, step=0.1, atol = 1e-6, max_iterations=500):
    """Gets the current joint angles, current pose and the final pose.
        Calculates the error and updates the joint angles until the error is less than the tolerance.
    """
    delta_A = (A_final - A_current)  #error
    iterations = 0
    while np.max(np.abs(delta_A)) > atol:
        J_q = J_lambdified(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        J_q = J_q/np.linalg.norm(J_q)   #normalizing to avoid larger angles
        delta_q = np.linalg.pinv(J_q) @ (delta_A*step)
        q = q + delta_q
        A_current = A_lambdified(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        delta_A = (A_final - A_current)
        iterations += 1

        if iterations > max_iterations:
            break

    return q, A_current

class InverseKinematics:
    def __init__(self):  
        self.jointangles = [q1, q2, q3, q4, q5, q6, q7]
        #dh parameters
        self.alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2]
        self.a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088]
        self.d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107]
        self.DK = self.dh_calculation()    
        DK = self.DK[0:3, 0:4]
        self.A = DK.transpose().reshape(12, 1)
        self.J = self.A.jacobian(self.jointangles)
        self.A_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), self.A, 'numpy'), nopython=True)       #issues in no_python argument
        self.J_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), self.J, 'numpy'), nopython=True)
        self.jointstate = JointState()   
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.pose_callback)
        self.joint_angles_publisher = rospy.Publisher("/goal_states", JointState, queue_size = 10)
        
    def pose_callback(self, msg):
        """Callback function for the subscriber.
           Gets the current joint angles and calculates FK and IK.
           publishes the joint angles to the /goal_states topic.
        """
        self.current_joint_position = np.array(msg.position)
        self.desired_joint_position = self.inverse_kinematics()
        #publishing the returned list/array as a JointState message
        self.jointstate.position = self.desired_joint_position.flatten()
        self.joint_angles_publisher.publish(self.jointstate)
        
    def dh_calculation(self):
        DK = eye(4)
        for i, (alpha, a, d, theta) in enumerate(zip(reversed(self.alpha), reversed(self.a), reversed(self.d), reversed(self.jointangles))):
            dh_between_frames = Matrix([[cos(theta), -sin(theta), 0, a],
                                        [cos(alpha)*sin(theta), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                                        [sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d],
                                        [0, 0, 0, 1]])
            DK = dh_between_frames @ DK
        return DK
    
    def inverse_kinematics(self):  
        #just for testing       
        q_current = self.current_joint_position.reshape(7, 1)   #gets the current jointangles from /joint_states
        A_current = self.A_lambdified(*(q_current).flatten())   
        #desired pose is assumed for now, which increases all jointangles by factor of 1
        random_pose = q_current + 1
        A_final = self.A_lambdified(*(random_pose).flatten())  #final pose to be reached
        q_final, A_final = incremental_ik(q_current, A_current, A_final, self.A_lambdified, self.J_lambdified)
        return q_final
    
if __name__ == "__main__":
    rospy.init_node("inverse_kinematics")
    ik = InverseKinematics()
    while not rospy.is_shutdown():
        rospy.spin()