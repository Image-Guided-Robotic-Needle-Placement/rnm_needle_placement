#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np

# Default initial joint positions
initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Default target joint angles
target_joint_angles = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  

# Interrupt flag for stopping the current motion and starting a new one
interrupt_trajectory = False

# Robot velocity, acceleration and jerk limits
max_velocity = 2.5*0.0001  # rad/s
max_acceleration = 25*0.00001  # rad/s²
max_jerk = 0.00001  # rad/s³, value to be adjusted based on your robot's specifications

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
    return np.dot(coeffs, powers)

# Joint States Callback
def joint_states_callback(msg):
    global initial_positions
    initial_positions = msg.position

def move_to_target(t0):
    global target_joint_angles, tf, interrupt_trajectory

    # Reset the interrupt flag at the start of the movement
    interrupt_trajectory = False

    # Calculate the necessary time to complete the motion (tf) for each joint
    tf_list = []
    quintic_coefficients = []  # Store the coefficients for each joint

    for joint_index in range(len(joint_names)):
        q0 = initial_positions[joint_index]
        qf = target_joint_angles[joint_index]

        delta_q = abs(qf - q0)

        # Assuming max acceleration and jerk, calculate time to reach max velocity
        # and adjust it if the total motion delta_q is too small
        tf_joint = max(np.sqrt(max_velocity/max_acceleration), np.cbrt(max_velocity/max_jerk))
        tf_joint = min(tf_joint, np.sqrt(4 * delta_q / max_acceleration))

        tf_list.append(tf_joint)

        # Calculate quintic coefficients for each joint
        conditions = {
            'q0': q0,
            'qf': qf,
            'v0': 0.0,
            'vf': 0.0,
            'a0': 0.0,
            'af': 0.0,
        }
        coeffs = calculate_quintic_coefficients(t0, tf_joint, **conditions)
        quintic_coefficients.append(coeffs)

    # Use the maximum time required as tf
    tf = max(tf_list)

    for t in np.arange(t0, tf, 0.1):
        if rospy.is_shutdown() or interrupt_trajectory:
            break

        joint_positions = Float64MultiArray()
        for joint_index, joint_name in enumerate(joint_names):
            joint_positions.data.append(polyval(t, quintic_coefficients[joint_index]))

        pub.publish(joint_positions)
        rate.sleep()

# Main function
if __name__ == '__main__':
    rospy.init_node('joint_trajectory_node')

    joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

    t0 = 0.0
    tf = None  # Will be calculated in move_to_target

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        # Read the target joint angles from the parameter server
        new_target_joint_angles = rospy.get_param('/target_joint_angles', None)

        if new_target_joint_angles is not None and new_target_joint_angles != target_joint_angles:
            target_joint_angles = new_target_joint_angles
            interrupt_trajectory = True
            move_to_target(t0)
