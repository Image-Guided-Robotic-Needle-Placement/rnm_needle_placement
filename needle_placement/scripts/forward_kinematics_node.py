#!/usr/bin/env python3 

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_matrix

def forwardKinematics(joint_angles):
    # Extract joint angles
    theta1, theta2, theta3, theta4, theta5, theta6, theta7 = joint_angles

    # Transformation matrices
    T01 = np.array([[np.cos(theta1), -np.sin(theta1), 0, 0],
                    [np.sin(theta1), np.cos(theta1), 0, 0],
                    [0, 0, 1, 0.333],
                    [0, 0, 0, 1]])

    # ... Define other transformation matrices T12, T23, ..., T67 ...
    T12 = np.array([[np.cos(theta2), 0, -np.sin(theta2), 0],
                    [np.sin(theta2), 0, np.cos(theta2), 0],
                    [0, -1, 0, 0],
                    [0, 0, 0, 1]])
    
    T23 = np.array([[np.cos(theta3), 0, np.sin(theta3), 0],
                    [np.sin(theta3), 0, -np.cos(theta3), 0],
                    [0, 1, 0, 0.316],
                    [0, 0, 0, 1]])
    
    T34 = np.array([[np.cos(theta4), 0, np.sin(theta4), 0.0825*np.cos(theta4)],
                    [np.sin(theta4), 0, -np.cos(theta4), 0.0825*np.sin(theta4)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
    
    T45 = np.array([[np.cos(theta5), 0, -np.sin(theta5), -0.0825*np.cos(theta5)],
                    [np.sin(theta5), 0, np.cos(theta5), -0.0825*np.sin(theta5)],
                    [0, -1, 0, 0.384],
                    [0, 0, 0, 1]])
    
    T56 = np.array([[np.cos(theta6), 0, np.sin(theta6), 0],
                    [np.sin(theta6), 0, -np.cos(theta6), 0],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
    
    T67 = np.array([[np.cos(theta7), 0, np.sin(theta7), 0.088*np.cos(theta7)],
                    [np.sin(theta7), 0, -np.cos(theta7), 0.088*np.sin(theta7)],
                    [0, 1, 0, 0],
                    [0, 0, 0, 1]])
    
    T7F = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.107],
                    [0, 0, 0, 1]])
    
    # TF_EE = np.array([[0.7071, -0.7071, 0, 0],
    #                 [0.7071, 0.7071, 0, 0],
    #                 [0, 0, 1, 0.1034],
    #                 [0, 0, 0, 1]])

    # Calculate final transformation matrix
    final_matrix = T01 @ T12 @ T23 @ T34 @ T45 @ T56 @ T67  @ T7F #@ TF_EE

    return final_matrix

def joint_states_callback(data):
    # Extract joint angles from JointState message
    joint_angles = data.position

    # Calculate the final transformation matrix
    final_matrix = forwardKinematics(joint_angles)

    # Create a Pose message
    pose_msg = Pose()
    pose_msg.position.x = final_matrix[0, 3]
    pose_msg.position.y = final_matrix[1, 3]
    pose_msg.position.z = final_matrix[2, 3]

    # Convert the transformation matrix to quaternion
    quaternion = quaternion_from_matrix(final_matrix)
    pose_msg.orientation.x = quaternion[0]
    pose_msg.orientation.y = quaternion[1]
    pose_msg.orientation.z = quaternion[2]
    pose_msg.orientation.w = quaternion[3]

    # Publish the Pose message to the end effector pose topic
    pose_pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('forward_kinematics_node')

    # Subscribe to the joint states topic
    rospy.Subscriber('/joint_states', JointState, joint_states_callback)

    # Create a publisher for the end effector pose topic
    pose_pub = rospy.Publisher('/end_effector_pose', Pose, queue_size=10)

    rospy.spin()