import numpy as np
import open3d as o3d

3
scan = o3d.io.read_point_cloud("finalfinalfinal.pcd")
# Visualize cloud and edit
vis = o3d.visualization.VisualizerWithEditing()
vis.create_window()
vis.add_geometry(scan)
vis.run()
vis.destroy_window()

picked_points = vis.get_picked_points()

if len(picked_points) != 2:
    print("Please only select 2 points! First should be the sphere point and second the entry point.")

scan_points = np.asarray(scan.points)
ball_point = scan_points[picked_points[0]]
entry_point = scan_points[picked_points[1]]
entry_point[2] += 0.15  # add height offset
f = open("D:/xzFACULTATE/SoSe23/rnm/needle_placement/entry_ball_points_scan.txt", 'a')
f.write(str(ball_point))
f.write(str(entry_point))
f.close()
print(ball_point)
print(entry_point)
