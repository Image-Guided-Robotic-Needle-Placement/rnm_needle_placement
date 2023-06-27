#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from numpy import linalg
from scipy.optimize import minimize
import numpy as np
import math
from numba import jit
from sympy import Matrix, lambdify
from sympy import symbols
q1, q2, q3, q4, q5, q6, q7 = symbols('q1 q2 q3 q4 q5 q6 q7')

current_matrix = directKinematics(initial_positions)
target_pose = np.array([1, 1, 1, 0, 0, 0])  # Modify as needed


def directKinematics(joint_angles):
    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = joint_angles
    # Dein vorheriger Code zur Berechnung der Transformationsmatrizen...

    T01 = Matrix([[cos(theta1), -sin(theta1), 0, 0],
                  [sin(theta1), cos(theta1), 0, 0],
                  [0, 0, 1, 0.333],
                  [0,0,0,1]])
    
    T12 = Matrix([[cos(theta2), -sin(theta2), 0, 0],
                    [0, 0, 1, 0],
                    [-sin(theta2), -cos(theta2), 0, 0],
                    [0, 0, 0, 1]])
    
    T23 = Matrix([[cos(theta3), -sin(theta3), 0, 0],
                    [0, 0, -1, -0.316],
                    [sin(theta3), cos(theta3), 0, 0],
                    [0, 0, 0, 1]])
    
    T34 = Matrix([[cos(theta4), -sin(theta4), 0, 0.0825],
                    [0, 0, -1, 0],
                    [sin(theta4), cos(theta4), 0, 0],
                    [0, 0, 0, 1]])
    
    T45 = Matrix([[cos(theta5), -sin(theta5), 0, -0.0825],
                    [0, 0, 1, 0.384],
                    [-sin(theta5), -cos(theta5), 0, 0],
                    [0, 0, 0, 1]])
    
    T56 = Matrix([[cos(theta6), -sin(theta6), 0, 0],
                    [0, 0, -1, 0],
                    [sin(theta6), cos(theta6), 0, 0],
                    [0, 0, 0, 1]])
    
    T67 = Matrix([[cos(theta7), -sin(theta7), 0, 0.088],
                    [0, 0, -1, -0.107],                              # 0.107 added to this frame
                    [sin(theta7), cos(theta7), 0, 0],
                    [0, 0, 0, 1]])
    
    # Calculate final transformation matrix
    final_matrix = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67 

    return Matrix(final_matrix)

# function to calculate incremental inverse kinematics
@jit
def incremental_ik(q, A, A_final, step=0.1, atol=1e-4):
    # Dein vorheriger Code zur Berechnung der inversen Kinematik...
    while True:
        delta_A = (A_final - A)
        if np.max(np.abs(delta_A)) <= atol:
            print("Joint angles calculated")
            break
        J_q = J_lamb(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        J_q = J_q / np.linalg.norm(J_q)  # normalize Jacobian
        
        # multiply by step to interpolate between current and target pose
        delta_q = np.linalg.pinv(J_q) @ (delta_A*step)
        
        q = q + delta_q
        A = A_lamb(q[0,0], q[1,0],q[2,0],q[3,0],q[4,0],q[5,0],q[6,0])
    return q, np.max(np.abs(delta_A))


initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Ändern Sie dies entsprechend Ihren Anforderungen

final_matrix = directKinematics(joint_angles)
A = final_matrix
A = A[0:3, 0:4]  
A = A.transpose().reshape(12,1) 

Q = Matrix(joint_angles)
J = A.jacobian(Q)  

A_lamb = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), A, 'numpy'))
J_lamb = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), J, 'numpy'))

limits = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973)
]

def pose_callback(msg):
    joint_angles = incremental_ik(initial_positions, current_matrix, target_pose)

    # Berechne die Gelenkwinkel mit der inversen Kinematik
    joint_angles = incremental_ik(initial_joint_angles, directKinematics(initial_joint_angles), np.array([msg.position.x, msg.position.y, msg.position.z]))

    # Veröffentliche die berechneten Gelenkwinkel
    joint_angles_msg = JointState()
    joint_angles_msg.position = joint_angles
    pub_joint_angles.publish(joint_angles_msg)

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node')

    initial_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    rospy.Subscriber('/target_pose', Pose, pose_callback)

    pub_joint_angles = rospy.Publisher('/joint_angles', JointState, queue_size=10)

    rospy.spin()