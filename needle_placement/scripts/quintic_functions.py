import numpy as np
import matplotlib.pyplot as plt

def position(coefficients, t): 
    """
    This function returns a joint angle at a given time

    Args:
        coefficients: The coefficients obtained from getJointTrajectoryCoefficients function
        t: The time from initial to final position

    Returns:
        The joint angle at specified time (in radian)
    """
    time = np.array([1,t,t**2,t**3,t**4,t**5])
    angle = np.dot(coefficients,time)
    return angle 

def velocity(coefficients, t): 
    """
    This function returns a joint velocity at a given time

    Args:
        coefficients: The coefficients obtained from getJointTrajectoryCoefficients function
        t: The time from initial to final position

    Returns:
        The joint velocity at specified time (in rad/s)
    """
    coefficients = coefficients[1:]
    time = np.array([1,2*t, 3*(t**2), 4*(t**3), 5*(t**4)])
    velocity = np.dot(coefficients,time)
    return velocity

def acceleration(coefficients, t): 
    """
    This function returns a joint acceleration at a given time

    Args:
        coefficients: The coefficients obtained from getJointTrajectoryCoefficients function
        t: The time from initial to final position

    Returns:
        The joint acceleration at specified time (in rad/s**2)
    """
    coefficients = coefficients[2:]
    time = np.array([2, 6*t, 12*(t**2), 20*(t**3)])
    acceleration = np.dot(coefficients,time)
    return acceleration

def getJointTrajectoryCoefficients(q_init, q_final, t):
    """
    This function calculates joint trajectory using quintic polynomial

    Args:
        q_init: initial joint position (current joint position) in radian
        q_final: final joint position (desired joint position)  in radian
        t: time to reach from q_init to q_final

    Returns: 
        The coefficients [c0,c1,c2,c3,c4,c5] for quintic trajectory
    """
    # Coefficient Matrix
    A = np.array([[1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 2, 0, 0, 0],
                [1, t, t**2, t**3, t**4, t**5],
                [0, 1, 2*t, 3*(t**2), 4*(t**3), 5*(t**4)],
                [0, 0, 2, 6*t, 12*(t**2), 20*(t**3)]])

    # Constant vector [q0, v0, a0, qf, vf, af]
    B = np.array([q_init, 0, 0, q_final, 0, 0])

    coefficients = np.linalg.solve(A, B)
    np.set_printoptions(suppress=True,precision=4)

    return coefficients

def calculateJointTrajectory(q_init, q_final, t, num_points):
    """
    Calculates Joint Trajectory for a single joint based on current and desired angle

    Args:
        q_init: initial joint position (current joint position) in radian
        q_final: final joint position (desired joint position)  in radian
        t: time to reach from q_init to q_final
        num_points: number of via-points

    Returns:
        Array of joint angles from q_init to q_final for a single joint
    """
    intervals = np.linspace(0, t, num_points)                 
    coefficients = getJointTrajectoryCoefficients(q_init,q_final,t)
    angles = []
    for i in range(num_points):
        angles.append(position(coefficients,intervals[i]))
    return np.array(angles)

def getJointsCoefficientMatrix(q_init_array, q_final_array, t):
    """
    This function calculates all the joint trajectories and returns 7 trajectory equations (as coefficients)

    Args:
        q_init_array: array of current 7 joint values
        q_final_array: array of desired 7 joint values
        t: time to move from current to desired joint values
    
    Returns:
        A 7x6 matrix of coefficients which represents 7 trajectory equations for each joint 
        and each equation has 6 coefficient
    """
    coefficient_matrix = None
    for i in range(7):
        # Check if coefficient_matrix is None to handle the first iteration
        if coefficient_matrix is None:
            coefficient_matrix = getJointTrajectoryCoefficients(q_init_array[i], q_final_array[i], t)
        else:
            coefficient_matrix = np.vstack((coefficient_matrix, getJointTrajectoryCoefficients(q_init_array[i], q_final_array[i], t)))
    
    return coefficient_matrix


def calculateRobotTrajectory(q_init_array, q_final_array, t, num_points):
    """
    This function calculates the joint angles that are passed to the robot controller to follow particular trajectory

    Args:
        q_init_array: array of current 7 joint values
        q_final_array: array of desired 7 joint values
        t: time to move from current to desired joint values
        num_points: number of via-points

    Returns:
        trajectory: A (num_points x 7) dimension matrix representing joint angles

    """
    intervals = np.linspace(0, t, num_points)
    coefficient_matrix = getJointsCoefficientMatrix(q_init_array,q_final_array,t)
    trajectory = None
    for i in range(num_points):
        if trajectory is None:
            trajectory = np.array([position(coefficients, intervals[i]) for coefficients in coefficient_matrix])
        else:
            trajectory = np.vstack((trajectory, np.array([position(coefficients, intervals[i]) for coefficients in coefficient_matrix])))

    return trajectory

def calculateSmoothRobotTrajectory(q_init_array, q_final_array, t, num_points):
    """
    This function calculates the joint angles that are passed to the robot controller to follow particular trajectory
    This function takess in account the maximum joint velocity limit of the robot

    Args:
        q_init_array: array of current 7 joint values
        q_final_array: array of desired 7 joint values
        t: time to move from current to desired joint values
        num_points: number of via-points

    Returns:
        trajectory: A (num_points x 7) dimension matrix representing joint angles

    """

    displacement = np.abs(np.array(q_final_array) - np.array(q_init_array))

    joint_moving_time = displacement/2.175
    joint_moving_time = np.round(joint_moving_time, 3)

    least_time = np.max(displacement)/2.175

    t = np.round(15 * least_time)  # duration
    num_points = int(1000 * t)

    intervals = np.linspace(0, t, num_points)
    coefficient_matrix = getJointsCoefficientMatrix(q_init_array,q_final_array,t)
    trajectory = None
    for i in range(num_points):
        if trajectory is None:
            trajectory = np.array([position(coefficients, intervals[i]) for coefficients in coefficient_matrix])
        else:
            trajectory = np.vstack((trajectory, np.array([position(coefficients, intervals[i]) for coefficients in coefficient_matrix])))

    return trajectory

def getVelocityProfile(q_init, q_final, t, num_points):
    """
    Calculates Joint Velocities for a single joint based on current angle, desired angle, t and num_points

    Args:
        q_init: initial joint position (current joint position) in radian
        q_final: final joint position (desired joint position)  in radian
        t: time to reach from q_init to q_final
        num_points: number of via-points

    Returns:
        Array of joint velocities from q_init to q_final for a single joint
    """
    intervals = np.linspace(0, t, num_points)                 
    coefficients = getJointTrajectoryCoefficients(q_init,q_final,t)
    velocities = []
    for i in range(num_points):
        velocities.append(velocity(coefficients,intervals[i]))
    return np.array(velocities)

def getAccelerationProfile(q_init, q_final, t, num_points):
    """
    Calculates Joint Accelerations for a single joint based on current angle, desired angle, t and num_points

    Args:
        q_init: initial joint position (current joint position) in radian
        q_final: final joint position (desired joint position)  in radian
        t: time to reach from q_init to q_final
        num_points: number of via-points

    Returns:
        Array of joint accelerations from q_init to q_final for a single joint
    """
    intervals = np.linspace(0, t, num_points)                 
    coefficients = getJointTrajectoryCoefficients(q_init,q_final,t)
    accelerations = []
    for i in range(num_points):
        accelerations.append(acceleration(coefficients,intervals[i]))
    return np.array(accelerations)

def plotAllVelocityAndAcceleration(q_init_array, q_final_array, t, num_points):
    """
    This function plots velocity vs time and acceleration vs time plots for each joint
    """
    time = np.linspace(0, t, num_points)   
    velocities = [getVelocityProfile(q_init, q_final, t, num_points) for q_init, q_final in zip(q_init_array, q_final_array)]
    accelerations = [getAccelerationProfile(q_init, q_final, t, num_points) for q_init, q_final in zip(q_init_array, q_final_array)]

    # Create subplots
    fig, axs = plt.subplots(7, 2, figsize=(10, 14))  # 7 rows, 2 columns

    # Iterate over each joint
    for i in range(7):
        # Plot velocities vs time on the first column
        axs[i, 0].plot(time, velocities[i])
        axs[i, 0].set_xlabel('Time')
        axs[i, 0].set_ylabel('Velocity')
        axs[i, 0].set_title(f'Joint {i+1} - Velocity')

    # Plot accelerations vs time on the second column
        axs[i, 1].plot(time, accelerations[i])
        axs[i, 1].set_xlabel('Time')
        axs[i, 1].set_ylabel('Acceleration')
        axs[i, 1].set_title(f'Joint {i+1} - Acceleration')

    # Adjust the spacing between subplots
    plt.tight_layout()

    # Display the plot
    plt.show()


if __name__ == '__main__':
    t = 4
    num_points = 30
    q_init_array = [-0.515, 0.434, 2.86, -1.09, 0.56, 2.3, 1.37]
    q_final_array = [-0.16, -0.67, 2.75, -0.94, 0.30, 3.07, 0.66]

    # To calculate the overall trajectory 
    trajectory = calculateSmoothRobotTrajectory(q_init_array,q_final_array,t,num_points)
    print(trajectory)

    # To plot velocity and acceleration for all joints
    plotAllVelocityAndAcceleration(q_init_array,q_final_array,t,num_points)

    
