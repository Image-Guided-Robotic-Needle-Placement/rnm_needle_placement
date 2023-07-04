#!/usr/bin/env python3

import numpy as np
import open3d as o3d
from os import listdir
from os.path import isfile, join

# Inspired from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html

# CT scan
reference_point_cloud_path = "C:/Users/Razvan/Downloads/Skeleton_Target.stl"
reference_point_cloud_mesh = o3d.io.read_triangle_mesh(reference_point_cloud_path)
reference_point_cloud = reference_point_cloud_mesh.sample_points_poisson_disk(100000)
# o3d.visualization.draw_geometries([reference_point_cloud_mesh])
# o3d.visualization.draw_geometries([reference_point_cloud])

# Captured pcds
point_clouds_path = "D:/xzFACULTATE/SoSe23/rnm/scanning"
point_clouds_files = []
for file in listdir(point_clouds_path):
    if file.endswith(".pcd"):
        point_clouds_files.append(file)

point_clouds_pcds = []

for point_cloud_file in point_clouds_files:
    temp_point_cloud = o3d.io.read_point_cloud(point_clouds_path + "/" + point_cloud_file)
    temp_point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    point_clouds_pcds.append(temp_point_cloud)
    # o3d.visualization.draw_geometries([temp_point_cloud])

# End-effector poses
endeffector_transformations = [np.array([[0.16495998, -0.03236812, 0.98576899, 0.28493194],
                                         [0.89696887, -0.41071458, -0.16358602, 0.10521373],
                                         [0.41016467, 0.91118924, -0.03871827, 0.68457447],
                                         [0, 0, 0, 1]]),
                               np.array([[0.12307247, 0.13130401, 0.98367292, 0.37113576],
                                         [0.93091952, -0.35872621, -0.06858822, 0.20056466],
                                         [0.34386336, 0.92416165, -0.16638279, 0.65951681],
                                         [0, 0, 0, 1]]),
                               np.array([[0.38675135, 0.03203948, 0.92162729, 0.45901381],
                                         [0.68377409, -0.68054129, -0.26328036, -0.02649904],
                                         [0.61877006, 0.7320089, -0.28510803, 0.67888965],
                                         [0, 0, 0, 1]]),
                               np.array([[0.50805224, -0.12620659, 0.85202982, 0.44904106],
                                         [0.47213001, -0.78655205, -0.39803156, -0.18571329],
                                         [0.72040001, 0.60448967, -0.34002361, 0.66373131],
                                         [0, 0, 0, 1]]),
                               np.array([[0.24948284, -0.19695306, 0.94813913, 0.26317212],
                                         [0.55409787, -0.77394559, -0.30656773, -0.14552408],
                                         [0.79418755, 0.60184526, -0.08395487, 0.69475573],
                                         [0, 0, 0, 1]])
                               ]
endeffector_transformations_inverse = []

for transformation in endeffector_transformations:
    endeffector_transformations_inverse.append(np.linalg.inv(transformation))

endeffector_transformations_inverse = np.array(endeffector_transformations_inverse)

# Hand-eye calibration
handeye_transformation = np.array([[-0.787647, 0.0228546, -0.615207, 0.00111628],
                                  [0.614978, -0.0192331, -0.787922, 0.0405029],
                                  [-0.0298236, -0.999441, 0.00107771, 0.00306807],
                                  [0, 0, 0, 1]])
handeye_transformation_inverse = np.linalg.inv(handeye_transformation)

# Stitch all pcds together after bringing them in the world coordinate system (robot-base)
stitched_point_cloud = o3d.geometry.PointCloud()
for index, point_cloud in enumerate(point_clouds_pcds):
    # o3d.visualization.draw_geometries([point_cloud])
    stitched_point_cloud += \
        ((point_cloud.transform(handeye_transformation)).transform(endeffector_transformations[index]))

# o3d.visualization.draw_geometries([stitched_point_cloud])
# o3d.io.write_point_cloud("teststiched.pcd", stitched_point_cloud)

# First apply global registration in order to get a good approximation and then refine the registration using ICP
def downsize_pcd(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    o3d.io.write_point_cloud("testdown.pcd", pcd_down)
    # pcd_down = pcd
    radius_feature = voxel_size * 5
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down,
                                            o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=10))
    return pcd_down, pcd_fpfh


voxel_size = 0.0005  # in meter
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