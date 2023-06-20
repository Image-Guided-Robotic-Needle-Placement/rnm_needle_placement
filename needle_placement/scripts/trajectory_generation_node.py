#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rnm_trajectory_function_numericaly import trajectory_generation

current_configuration = []


def joint_states_callback(data):
    global current_configuration
    current_configuration = data.position

def talker():
    rospy.init_node('trajectory_generation_node', anonymous=True)
    
    # Subscribe to the joint_states topic
    rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    
    pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(1000) # 1000 Hz
    
    desired_configuration = [-0.16 , -0.67 , 2.75 , -0.94 , 0.30, 3.07, 0.66] # in radians
    
    while not rospy.is_shutdown():
        # Check if current_configuration has been populated
        if current_configuration:
            trajectories = trajectory_generation(current_configuration, desired_configuration)
            for traj in trajectories:
                msg = Float64MultiArray()
                msg.data = traj
                pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
