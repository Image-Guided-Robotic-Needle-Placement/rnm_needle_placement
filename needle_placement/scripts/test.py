
import numpy as np

def rotation_matrix(phi, theta, psi):
    # Rotation matrix around x-axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])

    # Rotation matrix around y-axis
    Ry = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

    # Rotation matrix around z-axis
    Rz = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix
    R = np.dot(Rz, np.dot(Ry, Rx))

    return R

phi = np.radians(-152.492)  # Convert degrees to radians
theta = np.radians(-10.270)
psi = np.radians(13.539)

R = rotation_matrix(phi, theta, psi)
print(R)
