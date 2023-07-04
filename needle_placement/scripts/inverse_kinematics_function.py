import numpy as np
from sympy import symbols, Matrix, cos, sin, pi, eye
from numba import jit
from sympy import lambdify

        
# Define symbolic variables for 7 joint angles
q1, q2, q3, q4, q5, q6, q7 = symbols('q1 q2 q3 q4 q5 q6 q7')
    # Robot arm parameters
alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2]
a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088]
d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107]
jointangles = [q1, q2, q3, q4, q5, q6, q7]

# Compute the Direct Kinematics using the DH parameters
DK = eye(4)
for i, (alpha_i, a_i, d_i, theta_i) in enumerate(zip(reversed(alpha), reversed(a), reversed(d), reversed(jointangles))):
    dh_between_frames = Matrix([[cos(theta_i), -sin(theta_i), 0, a_i],
                                [cos(alpha_i)*sin(theta_i), cos(theta_i)*cos(alpha_i), -sin(alpha_i), -sin(alpha_i)*d_i],
                                [sin(theta_i)*sin(alpha_i), cos(theta_i)*sin(alpha_i), cos(alpha_i), cos(alpha_i)*d_i],
                                [0, 0, 0, 1]])
    DK = dh_between_frames @ DK

# Compute the Jacobian of the arm    
DK = DK[0:3, 0:4]
A = DK.transpose().reshape(12, 1)
J = A.jacobian(jointangles)

# Lambdify the direct kinematics and the Jacobian to use with numpy arrays
A_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), A, 'numpy'), nopython=True)
J_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), J, 'numpy'), nopython=True)

# Define a function to compute incremental inverse kinematics
@jit(nopython=True)
def incremental_ik(q, A_current, A_final, A_lambdified, J_lambdified, step=0.1, atol = 1e-6, max_iterations=500):
    delta_A = (A_final - A_current)
    iterations = 0
    while np.max(np.abs(delta_A)) > atol: # Run until the change in A is below the tolerance
        # Compute Jacobian at the current joint angles
        J_q = J_lambdified(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        J_q = J_q/np.linalg.norm(J_q) 
        # Compute the change in joint angles
        delta_q = np.linalg.pinv(J_q) @ (delta_A*step)
        q = q + delta_q
        # Update current pose
        A_current = A_lambdified(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        delta_A = (A_final - A_current)
        iterations += 1

        if iterations > max_iterations: # If maximum iterations reached, stop the loop
            break

    return q, A_current # Return final joint angles and final pose

# Function to perform inverse kinematics
def inverse_kinematics(current_joint_position, A_final):

    # Current joint position and pose
    q_current = np.array(current_joint_position).reshape(7, 1)
    A_current = A_lambdified(*q_current.flatten())

    # Compute the new joint angles using the incremental IK method
    q_final, _ = incremental_ik(q_current, A_current, A_final, A_lambdified, J_lambdified)
    q_final = q_final.flatten()
    for q in q_final:
        if q > 2*np.pi:
            q = q - 2*np.pi
        elif q < -2*np.pi:
            q = q + 2*np.pi
    print(q_final)
    return q_final

_=inverse_kinematics([0.0]*7, np.eye(4)[:3,:4].T.reshape(-1, 1))

if __name__ == "__main__":

    # Initial joint positions for testing
    current_joint_position = [0.33126478636661577, -0.8257679608411956, -1.0212689482370507, -2.6604452104735272, -0.21414480587762708, 1.936938619928904, -0.6195934342626068]

    # Desired final pose for testing
    A_final = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],
                       [0.19840159, -0.83371212, 0.51532603, -0.139],
                       [0.2377198, -0.46914648, -0.85052388, 0.266]]).T.reshape(-1, 1)  # Add your desired pose here

    # Compute the final joint angles to reach the desired pose
    final_joint_angles = inverse_kinematics(current_joint_position, A_final)

    # Print the final joint angles
    print("Final joint angles: ", final_joint_angles)
