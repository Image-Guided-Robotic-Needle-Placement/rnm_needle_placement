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
from os import listdir
from os.path import isfile, join



def compute_inverse_transformation(transformation):
    rotation_matrix = transformation[:3, :3]
    translation_vector = transformation[:3, 3]
    inverse_rotation = rotation_matrix.T
    inverse_translation = -translation_vector
    inverse_transformation = np.identity(4)
    inverse_transformation[:3, :3] = inverse_rotation
    inverse_transformation[:3, 3] = inverse_translation
    return inverse_transformation

reference_point_cloud_path = ...
reference_point_cloud = o3d.io.read_point_cloud(reference_point_cloud_path)

point_clouds_path = "/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/pointclouds/*.pcd"
point_clouds_files = [f for f in listdir(point_clouds_path) if isfile(join(point_clouds_path, f))]

endeffector_transformation = ... # get this pose from the DK
handeye_transformation = ... # get this pose from the handeye-calibration

endeffector_transformation_inverse = compute_inverse_transformation(endeffector_transformation)
handeye_transformation_inverse = compute_inverse_transformation(handeye_transformation)

point_clouds_pcds = []
for point_cloud_file in point_clouds_files:
    point_clouds_pcds.append(o3d.io.read_point_cloud(point_cloud_file))


stitched_point_cloud = o3d.geometry.PointCloud()
for point_cloud in point_clouds_pcds:
    stitched_point_cloud += ((point_cloud.transform(handeye_transformation_inverse)).transform(endeffector_transformation_inverse))

# first global registration and then ICP -Robin
