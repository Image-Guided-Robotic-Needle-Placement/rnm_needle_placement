#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Default initial joint positions
initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Default target joint angles
target_joint_angles = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  

# Robot velocity and acceleration limits (according to the Franka Emika documentation)
max_velocity = [2.175*0.001, 2.175*0.001, 2.175*0.001, 2.175*0.001, 2.61*0.001, 2.61*0.001, 2.61*0.001]  # rad/s
max_acceleration = [15*0.001, 7.5*0.001, 10*0.001, 12.5*0.001, 15*0.001, 20*0.001, 20*0.001]  # rad/s²

joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

# Initialize lists for storing time and joint data
time_history = []
position_history = [[] for _ in joint_names]
velocity_history = [[] for _ in joint_names]
acceleration_history = [[] for _ in joint_names]

# Initialize the plot
fig, axs = plt.subplots(3, 1, figsize=(10, 15))

# Flag to indicate if the initial positions are updated
initial_positions_updated = False

# Quintic Coefficients Calculation
def calculate_quintic_coefficients(t0, tf, q0, qf, v0, vf, a0, af):
    A = np.array([
        [t0**5, t0**4, t0**3, t0**2, t0, 1],
        [tf**5, tf**4, tf**3, tf**2, tf, 1],
        [5*t0**4, 4*t0**3, 3*t0**2, 2*t0, 1, 0],
        [5*tf**4, 4*tf**3, 3*tf**2, 2*tf, 1, 0],
        [20*t0**3, 12*t0**2, 6*t0, 2, 0, 0],
        [20*tf**3, 12*t0**2, 6*tf, 2, 0, 0]
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
    global initial_positions, initial_positions_updated
    if not initial_positions_updated:
        initial_positions = msg.position
        initial_positions_updated = True

def move_to_target(t0):
    global target_joint_angles

    # Read the current joint positions
    current_joint_positions = list(initial_positions)

    # Calculate the necessary time to complete the new motion (tf) for each joint
    tf_list = []
    for joint_index in range(len(joint_names)):
        q0 = current_joint_positions[joint_index]
        qf = target_joint_angles[joint_index]

        delta_q = abs(qf - q0)

        # Time needed to accelerate to max_velocity and decelerate to a stop
        t_accel_decel = 2 * max_velocity[joint_index] / max_acceleration[joint_index]

        if delta_q / max_velocity[joint_index] > t_accel_decel:
            # The joint can reach max_velocity
            # Time spent at max_velocity is the total time minus the acceleration and deceleration times
            t_max_velocity = delta_q / max_velocity[joint_index] - t_accel_decel
            tf_joint = t_accel_decel + t_max_velocity
        else:
            # The joint cannot reach max_velocity
            # Use the kinematic equation: delta_q = 0.5*a*t^2 to solve for time
            tf_joint = np.sqrt(4 * delta_q / max_acceleration[joint_index])

        tf_list.append(tf_joint)

    # Use the maximum time required as tf
    tf = max(tf_list)

    # For each time step...
    for t in np.arange(t0, tf, 0.1):
        if rospy.is_shutdown():
            break

        joint_positions = Float64MultiArray()

        # For each joint...
        for joint_index, joint_name in enumerate(joint_names):
            # Calculate the quintic coefficients and evaluate the polynomial
            conditions = {'q0': current_joint_positions[joint_index], 'qf': target_joint_angles[joint_index], 'v0': 0.0, 'vf': 0.0, 'a0': 0.0, 'af': 0.0}
            coeffs = calculate_quintic_coefficients(t0, tf, conditions['q0'], conditions['qf'], conditions['v0'], conditions['vf'], conditions['a0'], conditions['af'])

            # Calculate position, velocity, and acceleration
            position = polyval(t, coeffs)
            velocity = polyval(t, [0, coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4]])
            acceleration = polyval(t, [0, 0, 2 * coeffs[2], 6 * coeffs[3], 12 * coeffs[4], 20 * coeffs[5]])

            # Limit velocity and acceleration
            if velocity > max_velocity[joint_index]:
                velocity = max_velocity[joint_index]
            if velocity < -max_velocity[joint_index]:
                velocity = -max_velocity[joint_index]
            if acceleration > max_acceleration[joint_index]:
                acceleration = max_acceleration[joint_index]
            if acceleration < -max_acceleration[joint_index]:
                acceleration = -max_acceleration[joint_index]

            # Update the joint positions
            joint_positions.data.append(position)

            # Store the data for plotting
            position_history[joint_index].append(position)
            velocity_history[joint_index].append(velocity)
            acceleration_history[joint_index].append(acceleration)

        # Update the time history
        time_history.append(t)

        # Publish the joint positions
        pub.publish(joint_positions)
        rate.sleep()

# Plotting function
def update(i):
    # Clear the current plot
    for ax in axs:
        ax.clear()

    # Plot the new data
    for joint_index, joint_name in enumerate(joint_names):
        axs[0].plot(time_history, position_history[joint_index], label=joint_name)
        axs[1].plot(time_history, velocity_history[joint_index], label=joint_name)
        axs[2].plot(time_history, acceleration_history[joint_index], label=joint_name)

    # Set the plot labels
    axs[0].set(ylabel='Position (rad)')
    axs[1].set(ylabel='Velocity (rad/s)')
    axs[2].set(xlabel='Time (s)', ylabel='Acceleration (rad/s²)')

    # Set the plot titles
    axs[0].set_title('Joint Positions')
    axs[1].set_title('Joint Velocities')
    axs[2].set_title('Joint Accelerations')

    # Add legends
    axs[0].legend(loc='upper right')
    axs[1].legend(loc='upper right')
    axs[2].legend(loc='upper right')

    plt.tight_layout()

# ROS Node Initialization
rospy.init_node('panda_motion_planning', anonymous=True)
rate = rospy.Rate(100) # 100Hz

# ROS Subscriber Initialization
rospy.Subscriber('/joint_states', JointState, joint_states_callback)

# ROS Publisher Initialization
pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)

# Wait for the initial positions to be updated
while not initial_positions_updated and not rospy.is_shutdown():
    rospy.sleep(0.1)

# Start the motion at t0 = 0
move_to_target(t0=0)

# Start the animation
ani = FuncAnimation(fig, update, frames=range(len(time_history)), repeat=False)

plt.show()
