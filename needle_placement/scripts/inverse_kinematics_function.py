import numpy as np
from sympy import symbols, Matrix, cos, sin, pi, eye
from numba import jit
from sympy import lambdify

q1, q2, q3, q4, q5, q6, q7 = symbols('q1 q2 q3 q4 q5 q6 q7')

@jit(nopython=True)
def incremental_ik(q, A_current, A_final, A_lambdified, J_lambdified, step=0.1, atol = 1e-6, max_iterations=500):
    delta_A = (A_final - A_current)
    iterations = 0
    while np.max(np.abs(delta_A)) > atol:
        J_q = J_lambdified(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        J_q = J_q/np.linalg.norm(J_q) 
        delta_q = np.linalg.pinv(J_q) @ (delta_A*step)
        q = q + delta_q
        A_current = A_lambdified(q[0,0], q[1,0], q[2,0], q[3,0], q[4,0], q[5,0], q[6,0])
        delta_A = (A_final - A_current)
        iterations += 1

        if iterations > max_iterations:
            break

    return q, A_current

def inverse_kinematics(current_joint_position, A_final):
    alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2]
    a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088]
    d = [0.333, 0, 0.316, 0, 0.384, 0, 0.107]
    jointangles = [q1, q2, q3, q4, q5, q6, q7]
    
    DK = eye(4)
    for i, (alpha_i, a_i, d_i, theta_i) in enumerate(zip(reversed(alpha), reversed(a), reversed(d), reversed(jointangles))):
        dh_between_frames = Matrix([[cos(theta_i), -sin(theta_i), 0, a_i],
                                    [cos(alpha_i)*sin(theta_i), cos(theta_i)*cos(alpha_i), -sin(alpha_i), -sin(alpha_i)*d_i],
                                    [sin(theta_i)*sin(alpha_i), cos(theta_i)*sin(alpha_i), cos(alpha_i), cos(alpha_i)*d_i],
                                    [0, 0, 0, 1]])
        DK = dh_between_frames @ DK
        
    DK = DK[0:3, 0:4]
    A = DK.transpose().reshape(12, 1)
    J = A.jacobian(jointangles)
    A_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), A, 'numpy'), nopython=True)
    J_lambdified = jit(lambdify((q1, q2, q3, q4, q5, q6, q7), J, 'numpy'), nopython=True)

    q_current = np.array(current_joint_position).reshape(7, 1)
    A_current = A_lambdified(*q_current.flatten())
    q_final, _ = incremental_ik(q_current, A_current, A_final, A_lambdified, J_lambdified)

    return q_final.flatten()

if __name__ == "__main__":
    current_joint_position = [4.903597924155179e-06, -2.707226347986591e-06, 3.775922139404031e-06,  -0.3306003870171579, -2.4336162267601935e-06,  6.678026203132958e-07, 6.920757611439399e-07] # Make sure to use floating point numbers
    A_final = np.array([[0.95085546, 0.29124849, 0.10511047, 0.301],  
                        [0.19840159, -0.83371212, 0.51532603, -0.139],
                        [0.2377198, -0.46914648, -0.85052388, 0.266]]).T.reshape(-1, 1)  # Add your desired pose here

    final_joint_angles = inverse_kinematics(current_joint_position, A_final)
    print("Final joint angles: ", final_joint_angles)
