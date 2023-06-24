#!/usr/bin/env python3

import numpy as np
import open3d as o3d
from os import listdir
from os.path import isfile, join

# Inspired from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html

# CT scan
reference_point_cloud_path = ...
reference_point_cloud = o3d.io.read_point_cloud(reference_point_cloud_path)

# Captured pcds
point_clouds_path = "/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/pointclouds/*.pcd"
point_clouds_files = [f for f in listdir(point_clouds_path) if isfile(join(point_clouds_path, f))]
point_clouds_pcds = []

for point_cloud_file in point_clouds_files:
    point_clouds_pcds.append(o3d.io.read_point_cloud(point_cloud_file))

# End-effector poses
endeffector_transformations = np.array([])  # get the poses from the DK as np.array
endeffector_transformations_inverse = []

for transformation in endeffector_transformations:
    endeffector_transformations_inverse.append(np.linalg.inv(transformation))

endeffector_transformations_inverse = np.array(endeffector_transformations_inverse)

# Hand-eye calibration
handeye_transformation = np.array([])  # get this pose from the handeye-calibration as np.array
handeye_transformation_inverse = np.linalg.inv(handeye_transformation)

# Stitch all pcds together after bringing them in the world coordinate system (robot-base)
stitched_point_cloud = o3d.geometry.PointCloud()
for index, point_cloud in enumerate(point_clouds_pcds):
    stitched_point_cloud += \
        ((point_cloud.transform(handeye_transformation_inverse)).transform(endeffector_transformations_inverse[index]))

# First apply global registration in order to get a good approximation and then refine the registration using ICP
def downsize_pcd(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down,
                                            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


voxel_size = 0.05  # in meter
distance_threshold = voxel_size * 1.5

trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
stitched_point_cloud.transform(trans_init)
stitched_point_cloud_down, stitched_point_cloud_fpfh = downsize_pcd(stitched_point_cloud, voxel_size)
reference_point_cloud_down, reference_point_cloud_fpfh = downsize_pcd(reference_point_cloud, voxel_size)

# ToDo: adapt the parameters including the voxel size
# result.transformation for transformation
result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
    stitched_point_cloud_down, reference_point_cloud_down, stitched_point_cloud_fpfh, reference_point_cloud_fpfh,
    True, distance_threshold, o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
    3, [
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
            0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
            distance_threshold)
    ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))

# Refinement
distance_threshold = voxel_size * 0.4
result_refined = o3d.pipelines.registration.registration_icp(
    stitched_point_cloud_down, reference_point_cloud_down, distance_threshold, result.transformation,
    o3d.pipelines.registration.TransformationEstimationPointToPlane())