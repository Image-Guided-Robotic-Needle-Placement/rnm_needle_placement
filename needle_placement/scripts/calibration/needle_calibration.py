import numpy as np
from scipy.optimize import least_squares

# Inspired from "Which Pivot Calibration?" by Ziv Yaniv, https://dx.doi.org/10.1117/12.2081348

# ToDo: Load robot positions from file
# positions = np.loadtxt('robot_positions.txt')
# x = positions[:, 0]
# y = positions[:, 1]
# z = positions[:, 2]

positions = []
rotations_inverse = []

# Example for testing
x = np.array([1, -1, 0, 0, 0, 0])
y = np.array([0, 0, 0, 0, -1, 1])
z = np.array([0, 0, -1, 1, 0, 0])


# Function to compute the residuals for the sphere fit
def residuals(params):
    residuals_array = []
    cx, cy, cz, r = params
    for i in range(len(x)):
        residuals_array.append((x[i] - cx)**2 + (y[i] - cy)**2 + (z[i] - cz)**2 - r**2)
    return np.array(residuals_array)


# Initial guess for the sphere parameters
# ToDo: add a plausible value for the radius
initial_params = [0, 0, 0, 1]

# Perform sphere fit using least-squares
result = least_squares(residuals, initial_params)

centroid_x, centroid_y, centroid_z, radius = result.x
centroid_translation_world = np.array([centroid_x, centroid_y, centroid_z]) # sphere center in robot-base coordinate system
centroid_translations_ee = np.zeros((len(positions), 3))

for i in range(len(positions)):
    centroid_translations_ee[i] = rotations_inverse[i] * (centroid_translation_world - positions[i])

centroid_mean_translation_ee = np.mean(centroid_translations_ee, 0)

# Print the calibrated tip position
print("Calibrated Tip Position:")
print(centroid_mean_translation_ee)
