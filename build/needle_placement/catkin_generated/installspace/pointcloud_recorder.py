#!/usr/bin/env python3

import rospy
import rosbag
import numpy as np
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import Image, PointCloud2
from rospy import Duration
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import cv2
import open3d as o3d
import os
import pcl #not used here 


class PointCloudRecorderNode:
    def __init__(self):
        self.reached_position = False
        self.bridge = CvBridge()
        self.rgbimage_path = "/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/pointclouds"
        self.subscriber = rospy.Subscriber('/goal_reached', Bool, self.reached_position_callback)

    def reached_position_callback(self, data):

        timestamp = rospy.Time.now()

        if data.data == True:

            rospy.loginfo(str(timestamp) + ":Goal reached!")
            point_clouds = rospy.wait_for_message("/k4a/points2", PointCloud2, timeout=rospy.Duration(1))
            joint_msg = rospy.wait_for_message('/franka_state_controller/joint_states', JointState,
                                               timeout=rospy.Duration(1))

            # Save the joint states
            joint_msg = np.array(joint_msg.position)
            jointstate_array = joint_msg.flatten()
            f = open(
                "/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/pointclouds/jointstates.txt",
                "a")
            f.write(str(jointstate_array))
            f.close()

            # Generate point clouds as .pcd
            # https://answers.ros.org/question/255351/how-o-save-a-pointcloud2-data-in-python/
            gen = pc2.read_points(point_clouds, skip_nans=True)

            xyz = np.array([[0, 0, 0]])
            rgb = np.array([[0, 0, 0]])
            int_data = list(gen)

            for x in int_data:
                test = x[3]
                # cast float32 to int so that bitwise operations are possible
                s = struct.pack('>f', test)
                i = struct.unpack('>l', s)[0]
                # you can get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000) >> 16
                g = (pack & 0x0000FF00) >> 8
                b = (pack & 0x000000FF)
                # prints r,g,b values in the 0-255 range
                # x,y,z can be retrieved from the x[0],x[1],x[2]
                xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
                rgb = np.append(rgb, [[r, g, b]], axis=0)

            out_pcd = o3d.geometry.PointCloud()
            out_pcd.points = o3d.utility.Vector3dVector(xyz)
            out_pcd.colors = o3d.utility.Vector3dVector(rgb)
            o3d.io.write_point_cloud(
                "/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/pointclouds/scan1.pcd", out_pcd)

            rospy.loginfo("Pointcloud acquired")


if __name__ == "__main__":
    rospy.init_node("bag_recorder_node")
    node = PointCloudRecorderNode()
    rospy.spin()