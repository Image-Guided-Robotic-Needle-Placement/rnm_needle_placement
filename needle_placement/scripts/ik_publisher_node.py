#!/usr/bin/env python3 
from sympy import symbols, init_printing, Matrix, eye, sin, cos, pi
init_printing(use_unicode=True)

# reference from: https://gist.github.com/mlaves/a60cbc5541bd6c9974358fbaad9e4c51

# Importing necessary packages
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sympy import lambdify
from numba import jit

# create joint angles as symbols 
q1, q2, q3, q4, q5, q6, q7 = symbols('theta_1 theta_2 theta_3 theta_4 theta_5 theta_6 theta_7')
joint_angles = [q1, q2, q3, q4, q5, q6, q7]

# function to calculate direct kinematics
def directKinematics(joint_angles):
    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = joint_angles

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


final_matrix = directKinematics([q1, q2, q3, q4, q5, q6, q7]) # here we get final_matrix in symbolic form
# print(final_matrix) 
A = final_matrix

A = A[0:3, 0:4]  # crop last row
A = A.transpose().reshape(12,1)  # reshape to column vector A = [a11, a21, a31, ..., a34]

Q = Matrix(joint_angles)
J = A.jacobian(Q)  # compute Jacobian symbolically

# print(J)

# functions to convert pose and jacobian from symbolical to numerical form
A_lamb = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), A, 'numpy'))
J_lamb = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), J, 'numpy'))

# define joint limits for the Panda robot
limits = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973)
]

# Function to publish the array calculated from incremental inverse kinematics to /goal_states topic
def joint_state_publisher():
    rospy.init_node('ik_publisher_node', anonymous=True)
    pub = rospy.Publisher('goal_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # create initial pose
    q_init = np.array([l+(u-l)/2 for l, u in limits], dtype=np.float64).reshape(7, 1) # mid-points
    # print(q_init)
    A_init = A_lamb(*(q_init.flatten()))  # input: q_init Output: A_init (initial pose)  forward kinematics
    # print(A_init.reshape(3, 4, order='F')) # initial pose

    # generate random final pose within joint limits
    np.random.seed(0)

    # q_rand = np.array([np.random.uniform(l, u) for l, u in limits], dtype=np.float64).reshape(7, 1) # random joint angles between defined limits
    # # print(q_rand)
    # A_final = A_lamb(*(q_rand).flatten())  # input: q_rand Output: A_final (desired pose)  forward kinematics (desired pose is chosen randomly)
    # # print(A_final.reshape(3, 4, order='F')) # desired pose 

    desired_pose = np.array([[0.21582232, 0.95638546, 0.19684405, 0.55544547],
                           [0.91784019, -0.26748917, 0.29328985, 0.50407938],
                           [0.3331518, 0.11737288, -0.93553914, 0.33267676]])

    A_final = desired_pose.reshape(12, 1)

    q_final, _ = incremental_ik(q_init, A_init, A_final, atol=1e-6) # q_final is the required joint angles to reach A_final pose
    # print(q_final.flatten())
    q_final = q_final.flatten().tolist()
    # q_final.flatten()
    print(q_final)

    while not rospy.is_shutdown():
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

        # joint_state.position = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]  # Replace with your array of joint angles
        joint_state.position = q_final  # Replace with your array of joint angles

        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
