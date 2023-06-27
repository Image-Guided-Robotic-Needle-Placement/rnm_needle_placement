#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import argparse

# Default initial joint positions
initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

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

# Parse command line arguments for target joint angles
parser = argparse.ArgumentParser(description='Get target joint angles.')
parser.add_argument('-j', '--joint_angles', type=float, nargs='+', help='Target joint angles in radians.')
args = parser.parse_args()
if len(args.joint_angles) != 7:
    raise Exception("You must provide 7 joint angles as command line arguments!")
target_joint_angles = args.joint_angles

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

# Polynomial Derivative Evaluation
def polyder(t, coeffs):
    powers = np.array([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0])
    return np.dot(coeffs, powers)

# Polynomial Second Derivative Evaluation
def polyder2(t, coeffs):
    powers = np.array([20*t**3, 12*t**2, 6*t, 2, 0, 0])
    return np.dot(coeffs, powers)

# Move to Target
def move_to_target(t0):
    global initial_positions_updated
    global initial_positions

    if not initial_positions_updated:
        initial_positions_updated = True

    # Calculate coefficients for quintic polynomial
    tf = t0 + 5  # Assume motion takes 5 seconds
    coefficients = [calculate_quintic_coefficients(t0, tf, q0, qf, 0, 0, 0, 0) for q0, qf in zip(initial_positions, target_joint_angles)]

    # Generate motion over the interval [t0, tf]
    t = np.linspace(t0, tf, 500)  # 500 points of interpolation
    for ti in t:
        positions = [polyval(ti, coeff) for coeff in coefficients]
        velocities = [polyder(ti, coeff) for coeff in coefficients]
        accelerations = [polyder2(ti, coeff) for coeff in coefficients]

        # Update histories
        time_history.append(ti)
        for i in range(7):
            position_history[i].append(positions[i])
            velocity_history[i].append(velocities[i])
            acceleration_history[i].append(accelerations[i])

# Update function for the animation
def update(i):
    # Update position plot
    axs[0].cla()
    axs[0].set_title('Joint Positions')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Position (rad)')
    for j in range(7):
        axs[0].plot(time_history[:i+1], position_history[j][:i+1], label=joint_names[j])
    axs[0].legend()

    # Update velocity plot
    axs[1].cla()
    axs[1].set_title('Joint Velocities')
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Velocity (rad/s)')
    for j in range(7):
        axs[1].plot(time_history[:i+1], velocity_history[j][:i+1], label=joint_names[j])
    axs[1].legend()

    # Update acceleration plot
    axs[2].cla()
    axs[2].set_title('Joint Accelerations')
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Acceleration (rad/s²)')
    for j in range(7):
        axs[2].plot(time_history[:i+1], acceleration_history[j][:i+1], label=joint_names[j])
    axs[2].legend()

# Start the motion at t0 = 0
move_to_target(t0=0)

# Start the animation
ani = FuncAnimation(fig, update, frames=range(len(time_history)), repeat=False)

plt.show()
