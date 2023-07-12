#!/usr/bin/env python3

'''
Authors: Razvan-Andrei Draghici and Selvakumar Nachimuthu
'''

import numpy as np
import open3d as o3d
import copy
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

# Inspired from http://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html

# Sample the .pcd down and compute its features
def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
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
        pcd = o3d.io.read_point_cloud("/home/selva/Desktop/presentation/pointclouds/pointcloud%d.pcd" % i)

        ###  Manually filter the points clouds and save them as .ply's
        pcd.paint_uniform_color([0, 0, 1])
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()
        pcd = o3d.io.read_point_cloud("/home/selva/Desktop/presentation/pointclouds/cropped_%d.ply" % i)
        ###

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
    reference_pointcloud.scale(0.00095, [0, 0, 0])  # scale the .stl point-cloud such that it matches our scanned .pcds
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
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                10000 * voxel_size)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 0.99))
    return result


# ICP registration when the point-clouds are already in a similar pose
def refine_registration(source, target, voxel_size, trans):
    distance_threshold = 10 * voxel_size
    res = o3d.pipelines.registration.registration_icp(source, target, distance_threshold, trans)
    return res


####after registration
def compute_ball_points(ball_point, entry_point):
    # CS: local scan CS
    ball_point = np.append(ball_point, 1)   #homogeneous form
    entry_point = np.append(entry_point, 1)

    ball_point_scan_homogeneous = np.array(ball_point)
    entry_point_scan_homogeneous = np.array(entry_point)

    # Registration provided model -> our model
    registration_transformation = np.array(final_transform)

    #  Transformation end-effector -> need-tip
    needle_to_ee = np.eye(4)
    needle_to_ee[0:3, 3] = [-0.00156939339, 0.00076773158, 0.19702979428]

    # Ball- and entry-points in the global coordinate system
    # CS: global
    ball_point_global = np.matmul(registration_transformation, ball_point_scan_homogeneous)
    entry_point_global = np.matmul(registration_transformation, entry_point_scan_homogeneous)

    # Align the needle with the entry-ball axis
    # Direction vector Z
    direction_vector_Z = ball_point_global - entry_point_global
    direction_vector_Z = direction_vector_Z[0:3]
    direction_vector_Z = direction_vector_Z / np.linalg.norm(direction_vector_Z)

    # RX and RY are not uniquely determined by the needle.
    # To avoid collisions, we used values from a known, collision-free orientation of the robot.
    # Of course, the orientations have to be orthogonal to each other, so we only use little information from the
    # collision-free pose.
    # Direction vector Y
    direction_vector_Y = np.array([0.28769805,  -0.84302106, -(0.28769805 * direction_vector_Z[0] - 0.84302106 * direction_vector_Z[1])/direction_vector_Z[2]])
    direction_vector_Y = direction_vector_Y / np.linalg.norm(direction_vector_Y)

    # Direction vector X
    direction_vector_X = np.cross(direction_vector_Y, direction_vector_Z)

    needle_pose_ball = np.eye(4)
    needle_pose_entry = np.eye(4)

    needle_pose_entry[0:3, 0:3] = np.array([direction_vector_X, direction_vector_Y, direction_vector_Z])
    needle_pose_entry[0:3, 0:3] = np.transpose(needle_pose_entry[0:3, 0:3])
    needle_pose_entry[0:3, 3] = entry_point_global[0:3]

    needle_pose_ball[0:3, 0:3] = np.array([direction_vector_X, direction_vector_Y, direction_vector_Z])
    needle_pose_ball[0:3, 0:3] = np.transpose(needle_pose_ball[0:3, 0:3])
    needle_pose_ball[0:3, 3] = ball_point_global[0:3]

    # Compute the end-effector poses
    ee_pose_entry = np.matmul(needle_to_ee, needle_pose_entry)
    ee_pose_ball = np.matmul(needle_to_ee, needle_pose_ball)

    print("EE entry pose:", ee_pose_entry)
    print("EE ball pose:", ee_pose_ball)

    return ee_pose_entry, ee_pose_ball


if __name__ == "__main__":
    rospy.init_node("model_registration", anonymous=True)
    rospy.loginfo("Starting model registration node")

    #publishers
    entry_pose_pub = rospy.Publisher("/A_entry", Float64MultiArray , queue_size=1)
    ball_pose_pub = rospy.Publisher("/A_ball", Float64MultiArray, queue_size=1)
    
    #main code
    voxel_size = 0.007
    reference_path = "/home/selva/Downloads/scanning/Skeleton_Target.stl"
    pcds, pcds_down, pcds_fpfh = load_point_clouds(voxel_size=voxel_size)
    endeffector_transformations = np.load("/home/selva/Desktop/presentation/pointclouds/lab1_poses.npy")
    print("endeffector_transformations", endeffector_transformations)
    handeye_transformation = np.array([[0.67808914, -0.04218229, 0.7337682, 0.05999226],
                                       [-0.7349356, -0.04984826, 0.67630231, 0.01110063],
                                       [0.00804909, -0.99786562, -0.06480283, 0.04734731],
                                       [0, 0, 0, 1]])

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

        # Secondly, register the current pcd to the stitched pcd if the calibrations are not accurate
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

        # Thirdly, refine the stitching using ICP
        result = refine_registration(point_cloud, stitched_point_cloud, voxel_size, temp_registration.transformation)
        stitched_point_cloud += point_cloud
        o3d.visualization.draw_geometries([stitched_point_cloud])
        
    #selecting the entry and ball points using open3d visualizer
    stitched_down, scan_down, stitched_fpfh, scan_fpfh = prepare_dataset(voxel_size, reference_path, stitched_point_cloud)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(scan_down)
    vis.run()
    vis.get_cropped_geometry()
    vis.destroy_window()

    # Picked points indices in the point cloud
    picked_points = vis.get_picked_points()

    if len(picked_points) != 2:
        print("Please only select 2 points! First should be the sphere point and second the entry point.")

    scan_points = np.asarray(scan_down.points)
    ball_point = scan_points[picked_points[0]]
    entry_point = scan_points[picked_points[1]]
    entry_point[2] += 0.15  # add height offset
    f = open("/home/selva/catkin_ws/src/needle_placement/scripts/ball_points.txt", 'a')
    f.write(str(ball_point))
    f.write(str(entry_point))
    f.close()
    print('ball point:', ball_point)
    print('entry_point:', entry_point)
    o3d.io.write_point_cloud("final_stitched_down.pcd", stitched_down)
    o3d.io.write_point_cloud("finalfinalfinal.pcd", scan_down)
    final_transform = np.eye(4)

    # Global registration from the provided scan to the stitched point-cloud
    model_registration_global = execute_fast_global_registration(scan_down, stitched_down)
    final_transform = np.matmul(model_registration_global.transformation, final_transform)
    scan_down.transform(model_registration_global.transformation)
    global_fitness = model_registration_global.fitness
    index_global = 0
    while model_registration_global.fitness < 0.7 and index_global < 5:
        model_registration_global = execute_fast_global_registration(scan_down, stitched_down)
        if model_registration_global.fitness > global_fitness:
            final_transform = np.matmul(model_registration_global.transformation, final_transform)
            scan_down.transform(model_registration_global.transformation)
            global_fitness = model_registration_global.fitness
        index_global += 1

    # ICP refinement for better results
    model_registration_refined = refine_registration(scan_down, stitched_down, voxel_size, np.eye(4))
    final_transform = np.matmul(model_registration_refined.transformation, final_transform)
    o3d.visualization.draw_geometries([stitched_down])
    o3d.visualization.draw_geometries([scan_down])

    print(final_transform)

    # Stitch the stitched .pcd to the model for visualization
    stitched_down += scan_down.transform(model_registration_refined.transformation)
    o3d.visualization.draw_geometries([stitched_down])

    # waits for the subscriber to get ready and then publishes the message
    while entry_pose_pub.get_num_connections() == 0 or ball_pose_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    #poses
    ee_pose_entry, ee_pose_ball = compute_ball_points(ball_point, entry_point)
    print('ee_pose_entry: (after transformation)', ee_pose_entry)
    print('ee_pose_ball: (after transfromation)', ee_pose_ball)

    print("publishing message")
    #A_entry and A_ball should be published as float array type in a list like [rotations, translation]
    array_msg = Float64MultiArray()
    array_msg.layout.dim.append(MultiArrayDimension(label="rows", size=3, stride=4))
    array_msg.layout.dim.append(MultiArrayDimension(label="columns", size=4, stride=1))
    array_msg.layout.data_offset = 0
    array_msg.data = np.array(ee_pose_entry).reshape(-1).tolist()
    entry_pose_pub.publish(array_msg)

    ball_pose_msg = Float64MultiArray()
    array_msg.layout.dim.append(MultiArrayDimension(label="rows", size=3, stride=4))
    array_msg.layout.dim.append(MultiArrayDimension(label="columns", size=4, stride=1))
    array_msg.layout.data_offset = 0
    array_msg.data = np.array(ee_pose_entry).reshape(-1).tolist()
    entry_pose_pub.publish(array_msg)
    
    rospy.loginfo('Points Published')
    rospy.signal_shutdown('Points Published')

