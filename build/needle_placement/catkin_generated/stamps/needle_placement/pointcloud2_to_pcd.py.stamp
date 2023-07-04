#!/usr/bin/env python3

import rospy
import pcl
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, JointState
import os

class PointCloudConverterNode:
    def __init__(self):
        self.pc_subscriber = rospy.Subscriber('/k4a/points2', PointCloud2, self.pointcloud_callback)
        self.js_subscriber = rospy.Subscriber('/franka_state_controller/joint_states', JointState, self.jointstate_callback)
        self.pointcloud_count = 0
        self.latest_joint_state = None
        self.save_directory = '/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/joint_states'
        self.pointcloud_save_directory = '/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/pointclouds'

    def pointcloud_callback(self, msg):
        #msg.height = 480
        #msg.width = 640
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

    def jointstate_callback(self, msg):
        self.latest_joint_state = msg

    def save_joint_state(self):
        if hasattr(self, 'latest_joint_state'):
            file_name = os.path.join(self.save_directory, 'jointstate{}.txt'.format(self.pointcloud_count))
            with open(file_name, 'w') as f:
                f.write('Time: {}\n'.format(self.latest_joint_state.header.stamp))
                f.write('Position: {}\n'.format(self.latest_joint_state.position))
                f.write('Velocity: {}\n'.format(self.latest_joint_state.velocity))
                f.write('Effort: {}\n'.format(self.latest_joint_state.effort))
            rospy.loginfo("Saved JointState to: {}".format(file_name))
        else:
            rospy.logwarn('No joint state message')

if __name__ == '__main__':
    rospy.init_node('pointcloud_converter')
    converter = PointCloudConverterNode()
    rospy.spin()
