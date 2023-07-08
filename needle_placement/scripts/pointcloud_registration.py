import numpy as np
import open3d as o3d
import copy
import time
import os
import re

# Inspired from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html

def filter_pcd(pcd):
    pcd.paint_uniform_color([0, 0, 1])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    picked_points = vis.get_picked_points()
    scan_points = np.asarray(pcd.points)
    radius = 5

    pt_map = []
    for point in picked_points:
        camera_point = scan_points[point]
        camera = [camera_point[0], camera_point[1], camera_point[2]]
        test, pt_map_curr = pcd.hidden_point_removal(camera, radius)
        time.sleep(2)
        pt_map = pt_map + pt_map_curr

    pcd_visible = pcd.select_by_index(list(set(pt_map)))
    pcd_visible.paint_uniform_color([0, 0, 1])  # Blue points are visible points (to be kept).
    o3d.visualization.draw_geometries([pcd_visible])
    return pcd_visible
def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # pcd_down = filter_pcd(pcd_down)
    radius_normal = voxel_size * 10
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=100))

    radius_feature = voxel_size * 10
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def load_point_clouds(voxel_size=0.01):
    pcds = []
    pcds_down = []
    pcds_fpfh = []
    for i in range(1, 3):
        pcd = o3d.io.read_point_cloud("D:/xzFACULTATE/SoSe23/rnm/rnm_needle_placement-lab-final-fr/rnm_needle_placement-lab-final-fr/src/needle_placement/lab/pointclouds/scan%d.pcd" % i)
        # pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius= 10 * voxel_size, max_nn=30))
        pcd_down, pcd_fpfh = preprocess_point_cloud(pcd, voxel_size)
        pcds.append(pcd)
        pcds_down.append(pcd_down)
        pcds_fpfh.append(pcd_fpfh)
    return pcds, pcds_down, pcds_fpfh


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def generating_pointcloud_from_stl(reference_path):
    reference_pointcloud_mesh = o3d.io.read_triangle_mesh(reference_path)
    reference_pointcloud = reference_pointcloud_mesh.sample_points_poisson_disk(50000)
    reference_pointcloud.scale(0.00085, [0, 0, 0])
    # o3d.visualization.draw_geometries([reference_pointcloud])
    return reference_pointcloud


def prepare_dataset(voxel_size, reference_path, stitched_pcd):
    target = generating_pointcloud_from_stl(reference_path)
    source = stitched_pcd

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh


def execute_fast_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh):
    distance_threshold = voxel_size * 0.5
    # result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
    #     source_down, target_down, source_fpfh, target_fpfh,
    #     o3d.pipelines.registration.FastGlobalRegistrationOption(
    #         maximum_correspondence_distance=distance_threshold))
    # distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh)
    return result

def execute_global_registration(source_down, target_down, source_fpfh,
                                     target_fpfh, voxel_size):

    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True, 10000 * voxel_size,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),  # https://github.com/isl-org/Open3D/issues/2119
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                10000 * voxel_size)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.99))
    return result


def refine_registration(source, target, voxel_size, trans):
    distance_threshold = 10 * voxel_size
    res = o3d.pipelines.registration.registration_icp(source, target, distance_threshold, trans)
    return res


if __name__ == '__main__':

    voxel_size = 0.007
    reference_path = "C:/Users/Razvan/Downloads/Skeleton_Target.stl"
    pcds, pcds_down, pcds_fpfh = load_point_clouds(voxel_size=voxel_size)
    endeffector_transformations_inverse = []
    endeffector_transformations = np.load("D:/xzFACULTATE/SoSe23/rnm/rnm_needle_placement-lab-final-fr/rnm_needle_placement-lab-final-fr/src/needle_placement/scripts/new_poses.npy")

    for transformation in endeffector_transformations:
        endeffector_transformations_inverse.append(np.linalg.inv(transformation))

    endeffector_transformations_inverse = np.array(endeffector_transformations_inverse)

    # Hand-eye calibration
    # handeye_transformation = np.array([[-0.787647, 0.0228546, -0.615207, 0.00111628],
    #                                    [0.614978, -0.0192331, -0.787922, 0.0405029],
    #                                    [-0.0298236, -0.999441, 0.00107771, 0.00306807],
    #                                    [0, 0, 0, 1]])

    # handeye_transformation = np.array([[-0.999214, -0.00671978, -0.0389826, -0.034964],
    #                                    [0.0388317, 0.0215642, -0.999006, -0.0616634],
    #                                    [0.00755748, -0.99974, -0.0212874, 0.04776],
    #                                    [0, 0, 0, 1]])

    handeye_transformation = np.array([[ 0.67808914, -0.04218229,   0.7337682, 0.05999226],
                                       [-0.7349356 , -0.04984826,  0.67630231, 0.01110063],
                                       [ 0.00804909, -0.99786562, -0.06480283, 0.04734731],
                                       [0          , 0          , 0          , 1]])

    handeye_transformation_inverse = np.linalg.inv(handeye_transformation)

    # Stitch all pcds together after bringing them in the world coordinate system (robot-base)
    stitched_point_cloud = (pcds_down[0].transform(handeye_transformation)).transform(endeffector_transformations[0])

    stitched_copy_before = copy.deepcopy(stitched_point_cloud)
    stitched_copy_after = copy.deepcopy(stitched_point_cloud)
    for index, point_cloud in enumerate(pcds_down):

        if index == 0:
            continue

        # Firstly, bring the current point-cloud in the world coordinate system
        (point_cloud.transform(handeye_transformation)).transform(endeffector_transformations[index])
        stitched_copy_before += point_cloud
        o3d.visualization.draw_geometries([stitched_copy_before])

        stiched_fpfh = o3d.pipelines.registration.compute_fpfh_feature(stitched_point_cloud,
            o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 10, max_nn=100))

        # Secondly, do a global registration from the current point-cloud to the stiched one
        # global_registration = execute_fast_global_registration(point_cloud, stitched_point_cloud, pcds_fpfh[index], stiched_fpfh)
        # global_registration = execute_global_registration(point_cloud, stitched_point_cloud, pcds_fpfh[index], stiched_fpfh, voxel_size)
        # point_cloud.transform(global_registration.transformation)
        # stitched_copy_after += point_cloud
        # o3d.visualization.draw_geometries([stitched_copy_after])

        # Ignore outliers
        # if global_registration.inlier_rmse > 0.1:
        #     continue

        # Thirdly, do an ICP refinement
        result = refine_registration(point_cloud, stitched_point_cloud, voxel_size, np.eye(4))
        # if result.inlier_rmse > 0.1:
        #     continue

        stitched_point_cloud += point_cloud
        o3d.visualization.draw_geometries([stitched_point_cloud])

    stitched_down, scan_down, stitched_fpfh, scan_fpfh = prepare_dataset(voxel_size, reference_path, stitched_point_cloud)

    # o3d.visualization.draw_geometries([stitched_down])
    # o3d.visualization.draw_geometries([scan_down])
    # o3d.io.write_point_cloud("stitched_down_scaled.pcd", stitched_down)
    # o3d.io.write_point_cloud("scan_down.pcd", scan_down)

    model_registration_global = execute_global_registration(stitched_down, scan_down, stitched_fpfh, scan_fpfh, voxel_size)
    # model_registration_global = execute_fast_global_registration(stitched_down, scan_down, stitched_fpfh, scan_fpfh)
    model_registration_refined = refine_registration(stitched_down, scan_down, voxel_size, model_registration_global.transformation)
    o3d.visualization.draw_geometries([stitched_down])
    o3d.visualization.draw_geometries([scan_down])
    scan_down += stitched_down.transform(model_registration_refined.transformation)
    o3d.visualization.draw_geometries([scan_down])
