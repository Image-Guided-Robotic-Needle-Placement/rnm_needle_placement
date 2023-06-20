import numpy as np

def trajectory_generation(current_configuration, desired_configuration):

    q_max = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]        # in radians
    q_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]  # in radians

    for count, angle in enumerate(desired_configuration):
        if angle<q_min[count] or angle>q_max[count]:
            print("Desired configuration outside the joint limits: ", desired_configuration[count])
            return

    displacement = np.abs(np.array(desired_configuration) - np.array(current_configuration))

    joint_moving_time = displacement/2.175
    joint_moving_time = np.round(joint_moving_time, 3)

    least_time = np.max(displacement)/2.175

    duration = np.round(10 * least_time)
    num_points = 1000 #int(1000 * duration)

    coefficients = np.zeros((7, 6))
    for joint in range(7):
        coefficients[joint, 0] = current_configuration[joint]
        coefficients[joint, 3] = (10/(duration**3)) * (desired_configuration[joint] - current_configuration[joint])
        coefficients[joint, 4] = (-15/(duration**4)) * (desired_configuration[joint] - current_configuration[joint])
        coefficients[joint, 5] = (6/(duration**5)) * (desired_configuration[joint] - current_configuration[joint])

    intervals = np.linspace(0, duration, num_points)

    result_array = np.empty((num_points, 7))
    for i, t in enumerate(intervals):
        for joint in range(7):
            result_array[i, joint] = np.sum([coefficients[joint, j]*(t**j) for j in range(6)])

    return result_array, duration, num_points

current_configuration = [-0.515 , 0.434 , 2.86 , -1.09 , 0.56, 2.3, 1.37] # in radians
desired_configuration = [-0.16 , -0.67 , 2.75 , -0.94 , 0.30, 3.07, 0.66] # in radians

trajectories = trajectory_generation(current_configuration, desired_configuration)

print(trajectories)
