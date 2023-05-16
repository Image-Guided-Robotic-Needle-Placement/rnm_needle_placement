#Performing forward knematics for franka panda robot (converting joint positions to a robot space in cartesian space)
# current joint positions can be accessed from the topic /joint_states

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import tf
from sympy import symbols, init_printing, Matrix
from sympy import sin as s
from sympy import cos as c
from sympy import Abs


class InverseKinematics:
    def __init__(self):
        rospy.init_node("Inverse_kinematics", anonymous = True)
        self.pose_current_subscriber = rospy.Subscriber("/tf", Pose, self.jointstate_callback) # current pose
        self.pose_desired_subscriber = rospy.Subscriber("/tf", Pose, self.jointstate_callback) # desired pose
        self.jointstate_subscriber = rospy.Subscriber("/joint_states", JointState, self.jointstate_callback) # current joint states
        self.alpha = np.array([0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2])
        self.a = np.array([0, 0, 0, 0.0825, -0.0825, 0, 0])
        self.d = np.array([0.333, 0, 0.316, 0, 0.384, 0, 0.107])
        self.r = rospy.Rate(10)
    

    def jointstate_callback(self, msg):
        self.q = np.array(msg.position, dtype = np.float64)
        #publishing the joint angles we calculated using inverse kinematics
        joint_angles = self.inverse_kinematics()
        self.pose_pub = rospy.Publisher("/joint_angles", JointState, queue_size = 10)
        self.pose_pub.publish(joint_angles)
        self.r.sleep()

    def dh_between_frames(self, q, a, alpha, d):
        #general formula for transformation between two frames
        tf_matrix_between_frames = np.array([[np.cos(q), -np.sin(q), 0, a],
                                          [np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                                          [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha), np.cos(alpha), np.cos(alpha)*d],
                                          [0, 0, 0, 1]])
        return tf_matrix_between_frames

    def compute_dh_matrix(self):
        for i in range(0, 7):
            if i == 0:
                T_0_7 = self.dh_between_frames(self.q[i], self.a[i], self.alpha[i], self.d[i])
            else:
                T_0_7 = np.dot(T_0_7, self.dh_between_frames(self.q[i], self.a[i], self.alpha[i], self.d[i]))     
        return T_0_7          
    

    def incremental_ik(A, q_current, A_current, A_final, step=0.01):
        delta_A = (A_final - A_current)
        while max(Abs(delta_A)) > 0.01:
            J_q = J.evalf(subs={'theta_1': q_current[0],'theta_2': q_current[1], 'd_3': q_current[2]})
            # use pseudoinverse to solve over-determined problem
            # delta_A/10 is our linear interpolation between current and final pose
            delta_q = J_q.pinv() @ delta_A*step # @ is matrix product
            q_current = q_current + delta_q
            A_current = A.evalf(subs={'theta_1': q_current[0],'theta_2': q_current[1], 'd_3': q_current[2]})
            delta_A = (A_final - A_current)
        return q_current


    def inverse_kinematics(self):
        
        DK = self.compute_dh_matrix()

        # reshape to column vector A = [a11, a21, a31, ..., a34]
        A = DK[0:3, 0:4] # crop last row
        A = A.transpose().reshape(12,1)

        Q = Matrix(self.jointstate_subscriber)
        J = A.jacobian(Q)
 
        q_init = Matrix([self.jointstate_subscriber]).transpose()
        A_init = A.evalf(q_init[0], q_init[1],q_init[2],q_init[3], q_init[4],q_init[5],q_init[6])

        A_final = Matrix(self.pose_desired_subscriber)
        
        q_final = self.incremental_ik(A, q_init, A_init, A_final)
        A_final2 = A.evalf(subs={'theta_1': q_final[0],'theta_2': q_final[1],'d_3': q_final[2]})
        print(A_final2.reshape(4,3))


if __name__ == "__main__":
    fk = InverseKinematics()
    while not rospy.is_shutdown():
        rospy.spin()


    


