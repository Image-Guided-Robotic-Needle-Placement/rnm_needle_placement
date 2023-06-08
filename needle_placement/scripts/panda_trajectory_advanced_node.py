#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
from spatialmath.base import *
from spatialmath import SE3
import spatialmath.base.symbolic as sym
import roboticstoolbox as rtb
import numpy as np

def perform_trajectory():
    rospy.init_node('panda_trajectory_publisher')
    contoller_name='/position_joint_trajectory_controller/command'
    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)
    panda = rtb.models.URDF.Panda()
    argv = sys.argv[1:]                         
    panda_joints = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5',
                    'panda_joint6','panda_joint7']
    
    point = SE3(0.7, -0.5, 0.0) #this is the point where we want to move the end effector
    joint_angles = panda.ikine_LM(point).q
    goal_positions = joint_angles #this is the inverse kinematics solution for the point
    print("angles\n:", goal_positions)
    #goal_positions = [ float(argv[0]) , float(argv[1]) , float(argv[2]) ,float(argv[3] ) ,
    #                    float(argv[4]) , float(argv[5]) , float(argv[6])  ]
 
    rospy.loginfo("Goal Position set lets go ! ") 
    rospy.sleep(1)


    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = panda_joints
    trajectory_msg.points.append(JointTrajectoryPoint())
    trajectory_msg.points[0].positions = goal_positions
    trajectory_msg.points[0].velocities = [0.0 for i in panda_joints]
    trajectory_msg.points[0].accelerations = [0.0 for i in panda_joints]
    trajectory_msg.points[0].time_from_start = rospy.Duration(3)
    rospy.sleep(1)
    trajectory_publihser.publish(trajectory_msg)


if __name__ == '__main__':
    perform_trajectory()