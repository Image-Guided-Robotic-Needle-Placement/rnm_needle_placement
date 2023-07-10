import numpy as np
import open3d as o3d

for i in range(0, 3):
    pcd = o3d.io.read_point_cloud("D:/xzFACULTATE/SoSe23/rnm/needle_placement/lab/pointclouds/pointcloud%d.pcd" % i)
    pcd.paint_uniform_color([0, 0, 1])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

