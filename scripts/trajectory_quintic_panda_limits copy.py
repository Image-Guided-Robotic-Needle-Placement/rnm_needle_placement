#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Global flag to indicate if the initial positions are updated
initial_positions_updated = False

# Global variables for live plotting
fig, axs = plt.subplots(3, 1, figsize=(10, 15))
time_history = []
position_history = []
velocity_history = []
acceleration_history = []

# Robot velocity and acceleration limits (according to the Franka Emika documentation)
max_velocity = [2.175*0.01, 2.175*0.01, 2.175*0.01, 2.175*0.01, 2.61*0.01, 2.61*0.01, 2.61*0.01]  # rad/s
max_acceleration = [15*0.01, 7.5*0.01, 10*0.01, 12.5*0.01, 15*0.01, 20*0.01, 20*0.01]  # rad/s²

# Joint names
joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

# Callback function for /joint_states topic
def joint_states_callback(msg):
    global initial_positions_updated, initial_positions
    if not initial_positions_updated:
        initial_positions = msg.position
        initial_positions_updated = True
        rospy.loginfo('Initial positions received')

# Quintic coefficients calculation
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

# Polynomial evaluation

def polyval(x, coeffs):
    """Calculate the polynomial value using Horner's method."""
    result = 0
    for i in range(len(coeffs)):
        result = result * x + coeffs[i]
    return result

def move_to_target(t0):
    global target_joint_angles, time_history, position_history, velocity_history, acceleration_history

    # Read the current joint positions
    current_joint_positions = list(initial_positions)

    # Calculate the necessary time to complete the new motion (tf) for each joint
    tf_list = []
    for joint_index in range(7):
        joint_initial_position = current_joint_positions[joint_index]
        joint_target_position = target_joint_angles[joint_index]

        joint_travel_distance = np.abs(joint_target_position - joint_initial_position)

        # Max travel time according to max speed
        max_speed_travel_time = joint_travel_distance / max_velocity[joint_index]

        # Max travel time according to max acceleration
        max_accel_travel_time = np.sqrt(joint_travel_distance / max_acceleration[joint_index])

        # Choose the longer travel time
        tf = max(max_speed_travel_time, max_accel_travel_time)

        tf_list.append(tf)

    # Choose the longest tf for synchronization
    tf = max(tf_list)
    rospy.loginfo(f'Total travel time will be: {tf} seconds')

    # Calculate the quintic coefficients for each joint
    quintic_coeffs = []
    for joint_index in range(7):
        quintic_coeffs.append(
            calculate_quintic_coefficients(t0, tf, current_joint_positions[joint_index], target_joint_angles[joint_index], 0, 0, 0, 0)
        )

    # Reset the history
    time_history = []
    position_history = []
    velocity_history = []
    acceleration_history = []

    t = t0
    while t < tf:
        joint_positions = []
        joint_velocities = []
        joint_accelerations = []

        for joint_index in range(7):
            coeffs = quintic_coeffs[joint_index]

            position = polyval(t, coeffs)
            velocity = polyval(t, np.polyder(coeffs))
            acceleration = polyval(t, np.polyder(np.polyder(coeffs)))

            joint_positions.append(position)
            joint_velocities.append(velocity)
            joint_accelerations.append(acceleration)

        position_history.append(joint_positions)
        velocity_history.append(joint_velocities)
        acceleration_history.append(joint_accelerations)

        # Publish the joint positions to /panda/joint_position_controller/command topic
        msg = Float64MultiArray()
        msg.data = joint_positions
        pub.publish(msg)

        # Wait for the next control cycle
        t += dt
        rospy.sleep(dt)

# Live plot updating function
def update(frame):
    global time_history, position_history, velocity_history, acceleration_history

    # Clear the current plots
    for ax in axs:
        ax.cla()

    time_array = np.array(time_history)

    for joint_index in range(7):
        position_array = np.array([hist[joint_index] for hist in position_history])
        velocity_array = np.array([hist[joint_index] for hist in velocity_history])
        acceleration_array = np.array([hist[joint_index] for hist in acceleration_history])

        axs[0].plot(time_array, position_array, label=joint_names[joint_index])
        axs[1].plot(time_array, velocity_array, label=joint_names[joint_index])
        axs[2].plot(time_array, acceleration_array, label=joint_names[joint_index])

    axs[0].set_ylabel('Position (rad)')
    axs[1].set_ylabel('Velocity (rad/s)')
    axs[2].set_ylabel('Acceleration (rad/s²)')
    axs[2].set_xlabel('Time (s)')

    for ax in axs:
        ax.legend()
        ax.grid(True)

    plt.draw()

# Initialize ROS node
rospy.init_node('trajectory_planner')

# Initial positions of the robot
initial_positions = [0, 0, 0, 0, 0, 0, 0]

# Set the target joint angles
target_joint_angles = [0.785398, 0.785398, 0.785398, -0.785398, 0.785398, 1.5708, 0.785398]

# Control cycle duration (s)
dt = 0.001

# Subscribe to /joint_states topic
rospy.Subscriber('/joint_states', JointState, joint_states_callback)

# Wait until the initial joint positions are updated
rospy.loginfo('Waiting for initial joint positions...')
while not initial_positions_updated:
    rospy.sleep(0.001)

rospy.loginfo('Initial joint positions: ' + str(initial_positions))

# Publish joint positions to /panda/joint_position_controller/command topic
pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)

# Start the motion
move_to_target(0)

# Plot the results
ani = FuncAnimation(fig, update, frames=np.arange(0, len(time_history)), interval=20)
plt.show()



#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Global flag to indicate if the initial positions are updated
initial_positions_updated = False

# Global variables for live plotting
fig, axs = plt.subplots(3, 1, figsize=(10, 15))
time_history = []
position_history = []
velocity_history = []
acceleration_history = []

# Robot velocity and acceleration limits (according to the Franka Emika documentation)
max_velocity = [2.175*0.001, 2.175*0.001, 2.175*0.001, 2.175*0.001, 2.61*0.001, 2.61*0.001, 2.61*0.001]  # rad/s
max_acceleration = [15*0.001, 7.5*0.001, 10*0.001, 12.5*0.001, 15*0.001, 20*0.001, 20*0.001]  # rad/s²

# Joint names
joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

# Callback function for /joint_states topic
def joint_states_callback(msg):
    global initial_positions_updated, initial_positions
    if not initial_positions_updated:
        initial_positions = msg.position
        initial_positions_updated = True
        rospy.loginfo('Initial positions received')

# Quintic coefficients calculation
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

# Polynomial evaluation

def polyval(x, coeffs):
    """Calculate the polynomial value using Horner's method."""
    result = 0
    for i in range(len(coeffs)):
        result = result * x + coeffs[i]
    return result

def move_to_target(t0):
    global target_joint_angles, time_history, position_history, velocity_history, acceleration_history

    # Read the current joint positions
    current_joint_positions = list(initial_positions)

    # Calculate the necessary time to complete the new motion (tf) for each joint
    tf_list = []
    for joint_index in range(7):
        joint_initial_position = current_joint_positions[joint_index]
        joint_target_position = target_joint_angles[joint_index]

        joint_travel_distance = np.abs(joint_target_position - joint_initial_position)

        # Max travel time according to max speed
        max_speed_travel_time = joint_travel_distance / max_velocity[joint_index]

        # Max travel time according to max acceleration
        max_accel_travel_time = np.sqrt(joint_travel_distance / max_acceleration[joint_index])

        # Choose the longer travel time
        tf = max(max_speed_travel_time, max_accel_travel_time)

        tf_list.append(tf)

    # Choose the longest tf for synchronization
    tf = max(tf_list)
    rospy.loginfo(f'Total travel time will be: {tf} seconds')

    # Calculate the quintic coefficients for each joint
    quintic_coeffs = []
    for joint_index in range(7):
        quintic_coeffs.append(
            calculate_quintic_coefficients(t0, tf, current_joint_positions[joint_index], target_joint_angles[joint_index], 0, 0, 0, 0)
        )

    # Reset the history
    time_history = []
    position_history = []
    velocity_history = []
    acceleration_history = []

    t = t0
    while t < tf:
        joint_positions = []
        joint_velocities = []
        joint_accelerations = []

        for joint_index in range(7):
            coeffs = quintic_coeffs[joint_index]

            position = polyval(t, coeffs)
            velocity = polyval(t, np.polyder(coeffs))
            acceleration = polyval(t, np.polyder(np.polyder(coeffs)))

            joint_positions.append(position)
            joint_velocities.append(velocity)
            joint_accelerations.append(acceleration)

        position_history.append(joint_positions)
        velocity_history.append(joint_velocities)
        acceleration_history.append(joint_accelerations)

        # Publish the joint positions to /panda/joint_position_controller/command topic
        msg = Float64MultiArray()
        msg.data = joint_positions
        pub.publish(msg)

        # Wait for the next control cycle
        t += dt
        rospy.sleep(dt)

# Live plot updating function
def update(frame):
    global time_history, position_history, velocity_history, acceleration_history

    # Clear the current plots
    for ax in axs:
        ax.cla()

    time_array = np.array(time_history)

    for joint_index in range(7):
        position_array = np.array([hist[joint_index] for hist in position_history])
        velocity_array = np.array([hist[joint_index] for hist in velocity_history])
        acceleration_array = np.array([hist[joint_index] for hist in acceleration_history])

        axs[0].plot(time_array, position_array, label=joint_names[joint_index])
        axs[1].plot(time_array, velocity_array, label=joint_names[joint_index])
        axs[2].plot(time_array, acceleration_array, label=joint_names[joint_index])

    axs[0].set_ylabel('Position (rad)')
    axs[1].set_ylabel('Velocity (rad/s)')
    axs[2].set_ylabel('Acceleration (rad/s²)')
    axs[2].set_xlabel('Time (s)')

    for ax in axs:
        ax.legend()
        ax.grid(True)

    plt.draw()

# Initialize ROS node
rospy.init_node('trajectory_planner')

# Initial positions of the robot
initial_positions = [0, 0, 0, 0, 0, 0, 0]

# Set the target joint angles
target_joint_angles = [0.785398, 0.785398, 0.785398, -0.785398, 0.785398, 1.5708, 0.785398]

# Control cycle duration (s)
dt = 0.001

# Subscribe to /joint_states topic
rospy.Subscriber('/joint_states', JointState, joint_states_callback)

# Wait until the initial joint positions are updated
rospy.loginfo('Waiting for initial joint positions...')
while not initial_positions_updated:
    rospy.sleep(0.001)

rospy.loginfo('Initial joint positions: ' + str(initial_positions))

# Publish joint positions to /panda/joint_position_controller/command topic
pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)

# Start the motion
move_to_target(0)

# Plot the results
ani = FuncAnimation(fig, update, frames=np.arange(0, len(time_history)), interval=20)
plt.show()
