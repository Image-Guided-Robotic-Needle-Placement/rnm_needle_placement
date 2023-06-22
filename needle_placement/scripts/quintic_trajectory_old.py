import numpy as np
from sympy import symbols, pprint

def calculate_quintic_trajectory (q_init,q_final,duration,num_points):
    # Defining a symbolic variable t
    t = symbols('t')

    # Calculating coefficients
    c0 = q_init
    c1 = 0
    c2 = 0
    c3 = (10/(duration**3)) * (q_final - q_init)
    c4 = (-15/(duration**4)) * (q_final - q_init)
    c5 = (6/(duration**5)) * (q_final - q_init)

    # Quintic Trajectory
    traj = c0 + c1*t + c2*(t**2) + c3*(t**3) + c4*(t**4) + c5*(t**5)

    intervals = np.linspace(0, duration, num_points)

    angles = []
    for i in intervals:
        angles.append(traj.subs(t,i))

    return angles

def calculate_all_joint_trajectories(current_configuration,desired_configuration, duration, num_points):
    trajectories = []
    for i in range(7):
        trajectories.append(calculate_quintic_trajectory(current_configuration[i], desired_configuration[i], duration, num_points))

    # transpose the multi-dimensional array
    all_trajectories = list(map(list, zip(*trajectories)))

    # convert to numpy array
    all_trajectories = np.array(all_trajectories)

    # round-off all elements upto 3-decimal places
    all_trajectories = np.around(all_trajectories.astype(float), decimals=3)

    # print(all_trajectories.shape)
    # pprint(all_trajectories)

    return all_trajectories