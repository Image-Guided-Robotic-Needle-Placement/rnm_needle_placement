import numpy as np
import open3d as o3d
import copy
import time
import os
import re

'''def pass_through_filter(dic, pcd):
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        points[:,0]
        x_range = np.logical_and(points[:,0] >= dic["x"][0] ,points[:,0] <= dic["x"][1])
        y_range = np.logical_and(points[:,1] >= dic["y"][0] ,points[:,1] <= dic["y"][1])
        z_range = np.logical_and(points[:,2] >= dic["z"][0] ,points[:,2] <= dic["z"][1])

        pass_through_filter = np.logical_and(x_range,np.logical_and(y_range,z_range))

        pcd.points = o3d.utility.Vector3dVector(points[pass_through_filter])
        pcd.colors = o3d.utility.Vector3dVector(colors[pass_through_filter])

        return pcd'''


def load_point_clouds(voxel_size=0.02):
    pcds = []
    for i in range(1, 37):
        pcd = o3d.io.read_point_cloud("/home/selva/Downloads/scanning/final/pointclouds/pointcloud%d.pcd" % i)
        pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds


def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def draw_registration_result(source, target, transformation):
    print('drawing registration result')
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def generating_pointcloud_from_stl(reference_path):
    print('generating mesh')
    reference_pointcloud_mesh = o3d.io.read_triangle_mesh(reference_path)
    reference_pointcloud = reference_pointcloud_mesh.sample_points_poisson_disk(100000)
    # o3d.visualization.draw_geometries([reference_pointcloud])
    return reference_pointcloud


def prepare_dataset(voxel_size, reference_path, stitched_pcd):
    print('preparing dataset')
    source = generating_pointcloud_from_stl(reference_path)
    target = stitched_pcd

    trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
                             [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size=0.02):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
          % distance_threshold)
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result


def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, result_fast.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    return result


if __name__ == '__main__':

    reference_path = "/home/selva/Downloads/scanning/Skeleton_Target.stl"
    target_path = "/home/selva/Downloads/scanning/final/pointclouds"
    print(":: Load two point clouds and disturb initial pose.")

    pointcloud_files = []
    for file in os.listdir(target_path):
        if file.endswith(".pcd"):
            pointcloud_files.append(file)
    sorted_files = sorted(pointcloud_files, key=lambda x: int(re.search(r'\d+', x).group()))

    point_clouds_pcds = []

    for point_cloud_file in pointcloud_files:
        temp_point_cloud = o3d.io.read_point_cloud(target_path + "/" + point_cloud_file)
        temp_point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        point_clouds_pcds.append(temp_point_cloud)

    endeffector_transformations_inverse = []
    endeffector_transformations = np.load("/home/selva/catkin_ws/src/needle_placement/scripts/poses.npy")
    for transformation in endeffector_transformations:
        endeffector_transformations_inverse.append(np.linalg.inv(transformation))

    endeffector_transformations_inverse = np.array(endeffector_transformations_inverse)

    # Hand-eye calibration
    handeye_transformation = np.array([[-0.787647, 0.0228546, -0.615207, 0.00111628],
                                       [0.614978, -0.0192331, -0.787922, 0.0405029],
                                       [-0.0298236, -0.999441, 0.00107771, 0.00306807],
                                       [0, 0, 0, 1]])

    """handeye_transformation = np.array([[  -0.999214, -0.00671978,  -0.0389826,   -0.034964],
                                            [0.0388317,   0.0215642,   -0.999006,  -0.0616634],
                                            [0.00755748,    -0.99974,  -0.0212874,     0.04776],
                                            [0,           0,           0 ,          1]])"""
    handeye_transformation_inverse = np.linalg.inv(handeye_transformation)

    # Stitch all pcds together after bringing them in the world coordinate system (robot-base)
    stitched_point_cloud = o3d.geometry.PointCloud()
    for index, point_cloud in enumerate(point_clouds_pcds):
        stitched_point_cloud += \
            ((point_cloud.transform(handeye_transformation)).transform(endeffector_transformations[index]))

    o3d.visualization.draw_geometries([stitched_point_cloud])

    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(0.02, reference_path,
                                                                                         stitched_point_cloud)
    start = time.time()
    result_fast = execute_fast_global_registration(source_down, target_down,
                                                   source_fpfh, target_fpfh,
                                                   voxel_size=0.02)
    print("Fast global registration took %.3f sec.\n" % (time.time() - start))
    draw_registration_result(source_down, target_down, result_fast.transformation)
    print(result_fast)
    # 3result_fast = execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, 0.02)
    """result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        100, o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(0.02 * 1.5)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    print(result_ransac)
    draw_registration_result(source_down, target_down, result_ransac.transformation)
    print("RANSAC registration took %.3f sec.\n" % (time.time() - start))"""
    # refine registration
    # result_icp = refine_registration(source, target, source_fpfh, target_fpfh, 0.02)
    # print(result_icp)
    # print("ICP registration took %.3f sec.\n" % (time.time() - start))