#!/usr/bin/env python3

import rospy
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, JointState
from std_msgs.msg import Bool
import os
import numpy as np
import time

class PointCloudConverterNode:
    def __init__(self):

        #subscribers
        self.pc_subscriber = rospy.Subscriber('/k4a/points2', PointCloud2, self.pointcloud_callback) 
        self.js_subscriber = rospy.Subscriber('joint_states', JointState, self.jointstate_callback)
        self.trajectory_subscriber = rospy.Subscriber('/trajectory_done', Bool, self.trajectory_done_callback)
        
        self.pointcloud_count = 0
        self.latest_joint_state = None
        self.is_trajectory_done = False
        self.joint_states = []

        #file paths for storing point clouds and joint states
        self.save_directory = '/home/selva/Downloads/scanning/final/new/lab/jointstates'
        self.pointcloud_save_directory = '/home/selva/Downloads/scanning/final/new/lab/pointclouds'
        self.combined_file_path = '/home/selva/catkin_ws/src/needle_placement/scripts/combined_jointstates_lab.txt'
        self.npy_file_path = '/home/selva/catkin_ws/src/needle_placement/scripts/jointstates_lab.npy'
        self.poses_file_path = '/home/selva/catkin_ws/src/needle_placement/scripts/poses_lab.npy'

    def pointcloud_callback(self, msg): 
        self.current_pointcloud = msg

    def jointstate_callback(self, msg):
        self.latest_joint_state = msg

    def trajectory_done_callback(self, msg):  
        #upon trajectory completion, it calls the save pointcloud function with current pointcloud2 message
        self.is_trajectory_done = msg.data
        self.save_pointcloud(self.current_pointcloud)

    def save_pointcloud(self, msg):
        '''
        Pointcloud2 message is saved as pcd file also it saves the corresponding jointstates of the robot
        Args:
            msg: PointCloud2 message type
        '''
        if self.is_trajectory_done == True:
            points = pc2.read_points(msg, field_names=('x', 'y', 'z', 'rgb'), skip_nans=True)
            points_list = [(point[0], point[1], point[2], point[3]) for point in points]
            pcd = pcl.PointCloud_PointXYZRGB()
            pcd.from_list(points_list)

            file_name = 'pointcloud{}.pcd'.format(self.pointcloud_count)
            pointcloud_file_path = os.path.join(self.pointcloud_save_directory, file_name)
            pcl.save(pcd, pointcloud_file_path, format='pcd')
            rospy.loginfo("Saved PointCloud2 to: {}".format(file_name))

            self.save_joint_state()

            self.pointcloud_count += 1

    def save_joint_state(self):
        '''
        Saves the current jointstates.
        Once the desired number of pcds and jointstates are saved, it combines them and pass it to DK solver
        and saves the poses into .npy file
        '''
        if self.is_trajectory_done and hasattr(self, 'latest_joint_state'):
            file_name = os.path.join(self.save_directory, 'jointstate{}.txt'.format(self.pointcloud_count))
            with open(file_name, 'w') as f:
                f.write('Position: {}\n'.format(self.latest_joint_state.position))
            rospy.loginfo("Saved JointState to: {}".format(file_name))
            
            position_str = self.latest_joint_state.position
            position_values = tuple(float(val) for val in position_str)
            self.joint_states.append(position_values)
            
            if self.pointcloud_count == 15:   
                self.combine_joint_states()
                self.convert_to_poses()
                rospy.signal_shutdown('Finished processing all point clouds.')

        elif not self.is_trajectory_done:
            rospy.loginfo('Trajectory is not done yet. Skipping point cloud and joint state recording.')

    def combine_joint_states(self):
        with open(self.combined_file_path, 'w') as combined_file:
            for joint_state in self.joint_states:
                joint_state_str = ','.join(str(val) for val in joint_state)
                combined_file.write(joint_state_str + '\n')
        rospy.loginfo("Combined joint states saved as text file: {}".format(self.combined_file_path))

    def convert_to_poses(self):
        '''
        Does the forward kinematics (wrote again because of relative import error)
        '''
        alpha = np.array([0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2])
        a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0.088])
        d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0.107])

        def dh_between_frames(q, a, alpha, d):
            tf_matrix_between_frames = np.array([[np.cos(q), -np.sin(q), 0, a],
                                                  [np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                                                  [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
                                                  [0, 0, 0, 1]])
            return tf_matrix_between_frames

        def forward_kinematics_solver(q):
            T_0_7 = np.identity(4)
            for i in range(len(q)):
                T_0_7 = np.dot(T_0_7, dh_between_frames(q[i], a[i], alpha[i], d[i]))
            return T_0_7

        joint_states_np = np.array(self.joint_states)
        np.save(self.npy_file_path, joint_states_np)
        rospy.loginfo("Joint states saved as npy file: {}".format(self.npy_file_path))

        poses = []
        for q in joint_states_np:
            T_0_7 = forward_kinematics_solver(q)
            poses.append(T_0_7)

        poses_np = np.array(poses)
        np.save(self.poses_file_path, poses_np)
        rospy.loginfo("Poses saved as npy file: {}".format(self.poses_file_path))

if __name__ == '__main__':
    rospy.init_node('pointcloud_converter')
    converter = PointCloudConverterNode()
    rospy.spin()
