import numpy as np

# Define the points
entry_point = np.array([1, 2, 3])
ball_point = np.array([4, 5, 6])

# Compute the vector from A to B
AB = ball_point - entry_point

# Normalize AB to get the unit vector in the new z-direction
k = AB / np.linalg.norm(AB)

# Find a vector orthogonal to k for the new x-direction
# Start with an arbitrary vector
v = np.array([1, 0, 0])
# If v and k are not linearly independent, choose a different v
if np.allclose(v / np.linalg.norm(v), k):
    v = np.array([0, 1, 0])

# Subtract its projection onto k
u = v - np.dot(v, k) * k

# Normalize u to get the unit vector in the x-direction
i = u / np.linalg.norm(u)

# To find the unit vector in the y-direction, simply take the cross product of i and k
j = np.cross(i, k)

# Construct the orientation matrix
O = np.array([i, j, k]).T

A_entry = np.hstack((O, entry_point.reshape(-1, 1)))
A_ball = np.hstack((O, ball_point.reshape(-1, 1)))

print("Orientation matrix:\n", O)
print("A_entry:\n", A_entry)
print("A_ball:\n", A_ball)
