#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

# Default initial joint positions
initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Default target joint angles
target_joint_angles = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  

# Robot velocity and acceleration limits
max_velocity = 2.5*0.01  # 2.5*0.00008 rad/s 
max_acceleration = 25*0.01  # 25.0*0.00008 rad/s² 

# Quintic Coefficients Calculation
def calculate_quintic_coefficients(t0, tf, q0, qf, v0, vf, a0, af):
    A = np.array([
        [t0**5, t0**4, t0**3, t0**2, t0, 1],
        [tf**5, tf**4, tf**3, tf**2, tf, 1],
        [5*t0**4, 4*t0**3, 3*t0**2, 2*t0, 1, 0],
        [5*tf**4, 4*tf**3, 3*tf**2, 2*tf, 1, 0],
        [20*t0**3, 12*t0**2, 6*t0, 2, 0, 0],
        [20*tf**3, 12*tf**2, 6*tf, 2, 0, 0]
    ])

    b = np.array([q0, qf, v0, vf, a0, af])

    coefficients = np.linalg.solve(A, b)

    return coefficients

# Polynomial Evaluation
def polyval(t, coeffs):
    powers = np.array([t**5, t**4, t**3, t**2, t, 1])
    return np.dot(powers, coeffs)

# Joint States Callback
def joint_states_callback(msg):
    global initial_positions
    initial_positions = msg.position

def move_to_target(t0):
    global target_joint_angles, tf

    # Read the current joint positions
    current_joint_positions = list(initial_positions)

    # Calculate the necessary time to complete the new motion (tf) for each joint
    tf_list = []
    for joint_index in range(len(joint_names)):
        q0 = current_joint_positions[joint_index]
        qf = target_joint_angles[joint_index]

        delta_q = abs(qf - q0)

        # Time needed to accelerate to max_velocity and decelerate to a stop
        t_accel_decel = 2 * max_velocity / max_acceleration

        if delta_q / max_velocity > t_accel_decel:
            # The joint can reach max_velocity
            # Time spent at max_velocity is the total time minus the acceleration and deceleration times
            t_max_velocity = delta_q / max_velocity - t_accel_decel
            tf_joint = t_accel_decel + t_max_velocity
        else:
            # The joint cannot reach max_velocity
            # Use the kinematic equation: delta_q = 0.5*a*t^2 to solve for time
            tf_joint = np.sqrt(4 * delta_q / max_acceleration)

        tf_list.append(tf_joint)

    # Use the maximum time required as tf
    tf = max(tf_list)

    for t in np.arange(t0, tf, 0.01):
        if rospy.is_shutdown():
            break

        joint_positions = Float64MultiArray()
        for joint_index, joint_name in enumerate(joint_names):
            conditions = {
                'q0': current_joint_positions[joint_index],
                'qf': target_joint_angles[joint_index],
                'v0': 0.0,
                'vf': 0.0,
                'a0': 0.0,
                'af': 0.0,
            }
            coeffs = calculate_quintic_coefficients(t0, tf, conditions['q0'], conditions['qf'], conditions['v0'], conditions['vf'], conditions['a0'], conditions['af'])

            # Limit velocity and acceleration
            v = polyval(t, [0, coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4]])
            a = polyval(t, [0, 0, 2 * coeffs[2], 6 * coeffs[3], 12 * coeffs[4], 20 * coeffs[5]])

            if v > max_velocity:
                v = max_velocity
            if v < -max_velocity:
                v = -max_velocity
            if a > max_acceleration:
                a = max_acceleration
            if a < -max_acceleration:
                a = -max_acceleration

            joint_positions.data.append(polyval(t, coeffs))
        pub.publish(joint_positions)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_trajectory_node')

    joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

    t0 = 0.0
    tf = None  # Will be calculated in move_to_target

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    move_to_target(t0)  # Start the movement as soon as the node starts

    while not rospy.is_shutdown():
        # Read the target joint angles from the parameter server
        new_target_joint_angles = rospy.get_param('/target_joint_angles', None)

        if new_target_joint_angles is not None and new_target_joint_angles != target_joint_angles:
            target_joint_angles = new_target_joint_angles
            move_to_target(t0)
