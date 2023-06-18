#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from simple_pid import PID

# Default initial joint positions
initial_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Default target joint angles
target_joint_angles = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]  

# Create PID controllers for each joint
pid_controllers = [PID(0.1, 0.01, 0.001, setpoint=angle) for angle in target_joint_angles]

# Joint States Callback
def joint_states_callback(msg):
    global initial_positions
    initial_positions = msg.position

# Main function
if __name__ == '__main__':
    rospy.init_node('joint_trajectory_node')

    joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']

    rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():
        # Read the target joint angles from the parameter server
        new_target_joint_angles = rospy.get_param('/target_joint_angles', None)

        if new_target_joint_angles is not None and new_target_joint_angles != target_joint_angles:
            target_joint_angles = new_target_joint_angles
            # Update the setpoints of the PID controllers
            for i in range(7):
                pid_controllers[i].setpoint = target_joint_angles[i]

        joint_positions = Float64MultiArray()
        for i in range(7):
            # Update the PID controllers
            pid_controllers[i].output_limits = (initial_positions[i]-0.001, initial_positions[i]+0.001) # to limit the output
            control = pid_controllers[i](initial_positions[i])
            joint_positions.data.append(control)
        pub.publish(joint_positions)
        rate.sleep()
