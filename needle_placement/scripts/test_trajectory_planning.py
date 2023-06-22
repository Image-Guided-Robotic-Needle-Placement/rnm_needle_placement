import numpy as np
from quintic_trajectory_old import calculate_all_joint_trajectories

# Define the inputs
current_configuration = [0, 0, 0, 0, 0, 0, 0] # in radians
desired_configuration = [1, 2, 3, 4, 5, 6, 7]
duration = 5
num_points = 100

# Call the function
trajectories = calculate_all_joint_trajectories(current_configuration, desired_configuration, duration, num_points)

# Print the shape of the trajectories array (num_points,no_of_joints) # for panda, no_of_joints = 7
print(trajectories.shape)

# Print the trajectories array
print(trajectories)

