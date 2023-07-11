'''
Author: Razvan-Andrei Draghici
'''

import numpy as np
from scipy.optimize import least_squares
import open3d as o3d

# Inspired from "Which Pivot Calibration?" by Ziv Yaniv, https://dx.doi.org/10.1117/12.2081348

def load_poses(path):
    poses = []
    with open(path, 'r') as file:
        array = []
        for line in file:
            line = line.strip().strip('[').strip(']')
            if line:  # Check if the line is not empty
                array.append([float(num) for num in line.split()])
            else:  # Reached an empty line, store the current array
                if array:
                    poses.append(np.array(array))
                    array = []
    return poses


pose_file_path = "D:/xzFACULTATE/SoSe23/rnm/needle_poses.txt"
poses = load_poses(pose_file_path)

x = []
y = []
z = []
xyz = []

positions = []
rotations_inverse = []

# Extract positions and rotations from poses
for index, array in enumerate(poses):
    positions.append(np.array([array[0:3, 3]]) * 1000)  # transform to cmm
    rotations_inverse.append(np.transpose(array[0:3, 0:3]))
    x.append(array[0, 3] * 1000)
    y.append(array[1, 3] * 1000)
    z.append(array[2, 3] * 1000)
    xyz.append([array[0, 3] * 1000, array[1, 3] * 1000, array[2, 3] * 1000])

# Save points to .pcd for CloudCompare validation
xyz = np.array(xyz)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
o3d.io.write_point_cloud("calibration_point.pcd", pcd)

x = np.array([x]).reshape(1, len(x))
y = np.array([y]).reshape(1, len(y))
z = np.array([z]).reshape(1, len(z))

# Function to compute the residuals for the sphere fit
def residuals(params):
    residuals_array = []
    cx, cy, cz, r = params
    for i in range(len(x)):
        residuals_array.append((x[0, i] - cx)**2 + (y[0, i] - cy)**2 + (z[0, i] - cz)**2 - r**2)
    return np.array(residuals_array)


# Initial guess for the sphere parameters.
# Found either using a end-effector position and an estimation for the radius of the needle or with CloudCompare
initial_params = [101, -463, -31, 197]

# Perform sphere fit using least-squares
result = least_squares(residuals, initial_params)

centroid_x, centroid_y, centroid_z, radius = result.x
centroid_translation_world = np.array([centroid_x, centroid_y, centroid_z]) # sphere center in robot-base coordinate system
centroid_translations_ee = np.zeros((len(positions), 3))

# Compute the average translation from end-effector to needle-tip
for i in range(len(positions)):
    centroid_translations_ee[i] = np.matmul(rotations_inverse[i], (centroid_translation_world - positions[i]).reshape(3, 1)).ravel()

centroid_translations_ee = np.matrix(centroid_translations_ee)
centroid_mean_translation_ee = centroid_translations_ee.mean(0)

rmse = 0
# Compute the average translation from end-effector to needle-tip
for i in range(len(positions)):
    error_vec = centroid_mean_translation_ee - centroid_translations_ee[i]
    rmse += np.linalg.norm(error_vec)**2

# Print the calibrated tip position
print("Calibrated Tip Position:")
print(centroid_mean_translation_ee)
print("RMSE:")
print(np.sqrt(rmse / (3 * len(positions))))
