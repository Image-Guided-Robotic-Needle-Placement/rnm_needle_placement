import numpy as np
import open3d as o3d
import copy
import time

# Inspired from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html


# Filtering function currently NOT in use
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


# Sample the .pcd down and compute its features
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
    for i in range(0, 3):
        pcd = o3d.io.read_point_cloud("/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/cropped_%d.ply" % i)
        # pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius= 10 * voxel_size, max_nn=30))
        pcd_down, pcd_fpfh = preprocess_point_cloud(pcd, voxel_size)
        pcds.append(pcd)
        pcds_down.append(pcd_down)
        pcds_fpfh.append(pcd_fpfh)
    return pcds, pcds_down, pcds_fpfh


# Load the provided model as .pcd
def generating_pointcloud_from_stl(reference_path):
    reference_pointcloud_mesh = o3d.io.read_triangle_mesh(reference_path)
    reference_pointcloud = reference_pointcloud_mesh.sample_points_poisson_disk(50000)
    reference_pointcloud.scale(0.0009, [0, 0, 0])
    o3d.io.write_point_cloud("finalfinalfinal.pcd", reference_pointcloud)
    # o3d.visualization.draw_geometries([reference_pointcloud])
    return reference_pointcloud


def prepare_dataset(voxel_size, reference_path, stitched_pcd):
    target = generating_pointcloud_from_stl(reference_path)
    source = stitched_pcd

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh


def execute_fast_global_registration(source_down, target_down):
    distance_threshold = voxel_size * 0.5
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down, o3d.geometry.KDTreeSearchParamHybrid(radius= 10 * voxel_size, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down, o3d.geometry.KDTreeSearchParamHybrid(radius= 10 * voxel_size, max_nn=100))
    # result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
    #     source_down, target_down, source_fpfh, target_fpfh,
    #     o3d.pipelines.registration.FastGlobalRegistrationOption(
    #         maximum_correspondence_distance=distance_threshold))
    # distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh)
    return result

def execute_global_registration(source_down, target_down, voxel_size):

    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down, o3d.geometry.KDTreeSearchParamHybrid(radius= 10 * voxel_size, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down, o3d.geometry.KDTreeSearchParamHybrid(radius= 10 * voxel_size, max_nn=100))
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


# ICP registration when the point-clouds are already in a similar pose
def refine_registration(source, target, voxel_size, trans):
    distance_threshold = 10 * voxel_size
    res = o3d.pipelines.registration.registration_icp(source, target, distance_threshold, trans)
    return res


if __name__ == '__main__':

    voxel_size = 0.008
    reference_path = "/home/rnm-group2/Downloads/Skeleton_Target.stl"
    pcds, pcds_down, pcds_fpfh = load_point_clouds(voxel_size=voxel_size)
    endeffector_transformations_inverse = []
    endeffector_transformations = np.load("/home/rnm-group2/group2/rnm_needle_placement/src/needle_placement/lab/pointclouds/lab1_poses.npy")

    for transformation in endeffector_transformations:
        endeffector_transformations_inverse.append(np.linalg.inv(transformation))

    endeffector_transformations_inverse = np.array(endeffector_transformations_inverse)

    handeye_transformation = np.array([[0.67808914, -0.04218229, 0.7337682, 0.05999226],
                                       [-0.7349356, -0.04984826, 0.67630231, 0.01110063],
                                       [0.00804909, -0.99786562, -0.06480283, 0.04734731],
                                       [0, 0, 0, 1]])

    handeye_transformation_inverse = np.linalg.inv(handeye_transformation)

    # Stitch all pcds together after bringing them in the world coordinate system (robot-base)
    stitched_point_cloud = (pcds_down[0].transform(handeye_transformation)).transform(endeffector_transformations_inverse[0])
    stitched_copy_before = copy.deepcopy(stitched_point_cloud)
    stitched_copy_after = copy.deepcopy(stitched_point_cloud)
    # o3d.visualization.draw_geometries([stitched_copy_before])
    for index, point_cloud in enumerate(pcds_down):

        if index == 0:
            continue

        # Firstly, bring the current point-cloud in the world coordinate system
        (point_cloud.transform(handeye_transformation)).transform(endeffector_transformations_inverse[index])
        stitched_copy_before += point_cloud
        o3d.visualization.draw_geometries([stitched_copy_before])

        # o3d.visualization.draw_geometries([stitched_copy_after])
        temp_registration = execute_fast_global_registration(point_cloud, stitched_point_cloud)
        temp_fitness = temp_registration.fitness
        point_cloud.transform(temp_registration.transformation)
        index_temp = 0
        while temp_registration.fitness < 0.7 and index_temp < 5:
            temp_registration = execute_fast_global_registration(point_cloud, stitched_point_cloud)
            if temp_registration.fitness > temp_fitness:
                point_cloud.transform(temp_registration.transformation)
                temp_fitness = temp_registration.fitness
            index_temp += 1


        # Secondly, do an ICP refinement
        result = refine_registration(point_cloud, stitched_point_cloud, voxel_size, temp_registration.transformation)
        stitched_point_cloud += point_cloud
        o3d.visualization.draw_geometries([stitched_point_cloud])

    stitched_down, scan_down, stitched_fpfh, scan_fpfh = prepare_dataset(voxel_size, reference_path, stitched_point_cloud)
    o3d.io.write_point_cloud("final_stitched_down.pcd", stitched_down)

    # Global registration from the stitched point-cloud to the provided scan
    # model_registration_global_first = execute_global_registration(stitched_down, scan_down, stitched_fpfh, scan_fpfh, voxel_size)
    model_registration_global = execute_fast_global_registration(stitched_down, scan_down)
    stitched_down.transform(model_registration_global.transformation)
    global_fitness = model_registration_global.fitness
    index_global = 0
    while model_registration_global.fitness < 0.7 and index_global < 5:
        model_registration_global = execute_fast_global_registration(stitched_down, scan_down)
        if model_registration_global.fitness > global_fitness:
            stitched_down.transform(model_registration_global.transformation)
            global_fitness = model_registration_global.fitness
        index_global += 1

    # ICP refinement for better results
    model_registration_refined = refine_registration(stitched_down, scan_down, voxel_size, np.eye(4))
    o3d.visualization.draw_geometries([stitched_down])
    o3d.visualization.draw_geometries([scan_down])

    print(model_registration_refined.transformation)

    # Stitch the stitched .pcd to the model for visualization
    scan_down += stitched_down.transform(model_registration_refined.transformation)
    o3d.visualization.draw_geometries([scan_down])

