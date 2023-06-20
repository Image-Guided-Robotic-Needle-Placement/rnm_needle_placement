import numpy as np
import sys
from sympy import symbols, pprint

def trajectory_generation(current_configuration, desired_configuration):

    q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]        # in radians
    q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]  # in radians

    # v_max = max([2.1750, 2.1750, 2.1750, 2.1750, 2.61, 2.61, 2.61]) # in radians/s

    for count, angle in enumerate(desired_configuration):
        if angle<q_min[count] or angle>q_max[count]:
            print("Desired configuration outside the joint limits: ", desired_configuration[count])
            sys.exit()

    # Defining a symbolic variable t
    t = symbols('t')

    displacement_1 = abs(desired_configuration[0] - current_configuration[0])
    displacement_2 = abs(desired_configuration[1] - current_configuration[1])
    displacement_3 = abs(desired_configuration[2] - current_configuration[2])
    displacement_4 = abs(desired_configuration[3] - current_configuration[3])
    displacement_5 = abs(desired_configuration[4] - current_configuration[4])
    displacement_6 = abs(desired_configuration[5] - current_configuration[5])
    displacement_7 = abs(desired_configuration[6] - current_configuration[6])

    displacement = [displacement_1, displacement_2, displacement_3, displacement_4, displacement_5, displacement_6, displacement_7]
    # print("displacements: ", displacement)

    joint_moving_time = [displacement_1/2.175, displacement_2/2.175, displacement_3/2.175, displacement_4/2.175, displacement_5/2.175, displacement_6/2.175, displacement_7/2.175]
    joint_moving_time = [round(element, 3) for element in joint_moving_time]
    # print("Joint Moving Time: ", joint_moving_time)

    max_displacement = abs(max(displacement))
    min_displacement = abs(min(displacement))

    least_time = max_displacement/2.175
    # print("least_time: ", round(least_time,3))   # this is the minimum time that robot will take if it moves at maximum velocity (it cannot move faster than this)

    duration = round(10 * least_time)
    # print("duration: ",duration)

    num_points = 1000 * duration
    # print("num_points: ", num_points)

    # Calculating coefficients for joint 1
    j1_c0 = current_configuration[0]
    j1_c1 = 0
    j1_c2 = 0
    j1_c3 = (10/(duration**3)) * (desired_configuration[0] - current_configuration[0])
    j1_c4 = (-15/(duration**4)) * (desired_configuration[0] - current_configuration[0])
    j1_c5 = (6/(duration**5)) * (desired_configuration[0] - current_configuration[0])

    # Quintic Trajectory for joint 1
    traj1 = j1_c0 + j1_c1*t + j1_c2*(t**2) + j1_c3*(t**3) + j1_c4*(t**4) + j1_c5*(t**5)

    # Calculating coefficients for joint 2
    j2_c0 = current_configuration[1]
    j2_c1 = 0
    j2_c2 = 0
    j2_c3 = (10/(duration**3)) * (desired_configuration[1] - current_configuration[1])
    j2_c4 = (-15/(duration**4)) * (desired_configuration[1] - current_configuration[1])
    j2_c5 = (6/(duration**5)) * (desired_configuration[1] - current_configuration[1])

    # Quintic Trajectory for joint 2
    traj2 = j2_c0 + j2_c1*t + j2_c2*(t**2) + j2_c3*(t**3) + j2_c4*(t**4) + j2_c5*(t**5)

    # Calculating coefficients for joint 3
    j3_c0 = current_configuration[2]
    j3_c1 = 0
    j3_c2 = 0
    j3_c3 = (10/(duration**3)) * (desired_configuration[2] - current_configuration[2])
    j3_c4 = (-15/(duration**4)) * (desired_configuration[2] - current_configuration[2])
    j3_c5 = (6/(duration**5)) * (desired_configuration[2] - current_configuration[2])

    # Quintic Trajectory for joint 3
    traj3 = j3_c0 + j3_c1*t + j3_c2*(t**2) + j3_c3*(t**3) + j3_c4*(t**4) + j3_c5*(t**5)

    # Calculating coefficients for joint 4
    j4_c0 = current_configuration[3]
    j4_c1 = 0
    j4_c2 = 0
    j4_c3 = (10/(duration**3)) * (desired_configuration[3] - current_configuration[3])
    j4_c4 = (-15/(duration**4)) * (desired_configuration[3] - current_configuration[3])
    j4_c5 = (6/(duration**5)) * (desired_configuration[3] - current_configuration[3])

    # Quintic Trajectory for joint 4
    traj4 = j4_c0 + j4_c1*t + j4_c2*(t**2) + j4_c3*(t**3) + j4_c4*(t**4) + j4_c5*(t**5)

    # Calculating coefficients for joint 5
    j5_c0 = current_configuration[4]
    j5_c1 = 0
    j5_c2 = 0
    j5_c3 = (10/(duration**3)) * (desired_configuration[4] - current_configuration[4])
    j5_c4 = (-15/(duration**4)) * (desired_configuration[4] - current_configuration[4])
    j5_c5 = (6/(duration**5)) * (desired_configuration[4] - current_configuration[4])

    # Quintic Trajectory for joint 5
    traj5 = j5_c0 + j5_c1*t + j5_c2*(t**2) + j5_c3*(t**3) + j5_c4*(t**4) + j5_c5*(t**5)

    # Calculating coefficients for joint 6
    j6_c0 = current_configuration[5]
    j6_c1 = 0
    j6_c2 = 0
    j6_c3 = (10/(duration**3)) * (desired_configuration[5] - current_configuration[5])
    j6_c4 = (-15/(duration**4)) * (desired_configuration[5] - current_configuration[5])
    j6_c5 = (6/(duration**5)) * (desired_configuration[5] - current_configuration[5])

    # Quintic Trajectory for joint 6
    traj6 = j6_c0 + j6_c1*t + j6_c2*(t**2) + j6_c3*(t**3) + j6_c4*(t**4) + j6_c5*(t**5)

    # Calculating coefficients for joint 7
    j7_c0 = current_configuration[6]
    j7_c1 = 0
    j7_c2 = 0
    j7_c3 = (10/(duration**3)) * (desired_configuration[6] - current_configuration[6])
    j7_c4 = (-15/(duration**4)) * (desired_configuration[6] - current_configuration[6])
    j7_c5 = (6/(duration**5)) * (desired_configuration[6] - current_configuration[6])

    # Quintic Trajectory for joint 6
    traj7 = j7_c0 + j7_c1*t + j7_c2*(t**2) + j7_c3*(t**3) + j7_c4*(t**4) + j7_c5*(t**5)

    # print("traj1: ",traj1)
    # print("traj2: ",traj2)
    # print("traj3: ",traj3)
    # print("traj4: ",traj4)
    # print("traj5: ",traj5)
    # print("traj6: ",traj6)
    # print("traj7: ",traj7)

    intervals = np.linspace(0, duration, num_points)
    # print("Intervals: ",intervals)

    # angles = [angles1, angles2, angles3, angles4]

    # # print(angles1)
    # # print(angles2)
    # # print(angles3)
    # # print(angles4)

    # trajectory = np.vstack(tuple(angles))

    # # print(trajectory)

    # Create an empty array to store the values
    result_array = np.empty((num_points, 7))

    for i in range(len(intervals)):
        angles = np.array([traj1.subs(t,intervals[i]), traj2.subs(t,intervals[i]), traj3.subs(t,intervals[i]), traj4.subs(t,intervals[i]), traj5.subs(t,intervals[i]), traj6.subs(t,intervals[i]), traj7.subs(t,intervals[i])])
        result_array[i] = angles

    return result_array, duration, num_points


current_configuration = [-0.515 , 0.434 , 2.86 , -1.09 , 0.56, 2.3, 1.37] # in radians
desired_configuration = [-0.16 , -0.67 , 2.75 , -0.94 , 0.30, 3.07, 0.66] # in radians

trajectories = trajectory_generation(current_configuration, desired_configuration)

print(trajectories)