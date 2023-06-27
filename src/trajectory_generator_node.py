#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import numpy as np

# Default initial joint positions
initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Default target joint angles
target_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Interrupt flag for stopping the current motion and starting a new one
interrupt_trajectory = False

# Robot velocity and acceleration limits
max_velocity = 2.5*0.0001  # 2.5*0.00008 rad/s
max_acceleration = 25*0.00001  # 25.0*0.00008 rad/s²

# ... Dein restlicher Code bleibt gleich ...
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

def joint_states_callback(msg):
    global initial_positions
    initial_positions = msg.position

def joint_angles_callback(msg):
    global target_joint_angles, interrupt_trajectory
    target_joint_angles = msg.position
    interrupt_trajectory = True

# ... Dein restlicher Code bleibt gleich ...
def move_to_target(t0):
    global target_joint_angles, tf, interrupt_trajectory

    # Reset the interrupt flag at the start of the movement
    interrupt_trajectory = False

    # Calculate the necessary time to complete the motion (tf) for each joint
    tf_list = []
    for joint_index in range(len(joint_names)):
        q0 = initial_positions[joint_index]
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

    for t in np.arange(t0, tf, 0.1): # 0.01
        if rospy.is_shutdown() or interrupt_trajectory:
            break

        joint_positions = Float64MultiArray()
        for joint_index, joint_name in enumerate(joint_names):
            conditions = {
                'q0': initial_positions[joint_index],
                'qf': target_joint_angles[joint_index],
                'v0': 0.0,
                'vf': 0.0,
                'a0': 0.0,
                'af': 0.0,
            }
            coeffs = calculate_quintic_coefficients(t0, tf, conditions['q0'], conditions['qf'], conditions['v0'], conditions['vf'], conditions['a0'], conditions['af'])
            joint_positions.data.append(polyval(t, coeffs))
        pub.publish(joint_positions)
        rate.sleep()

# Main function

if __name__ == '__main__':
    rospy.init_node('joint_trajectory_node')

    joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

    t0 = 0.0
    tf = None  # Will be calculated in move_to_target

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    rospy.Subscriber('/joint_angles', JointState, joint_angles_callback)

    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        if interrupt_trajectory:
            move_to_target(t0)