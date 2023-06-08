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
    # Initialize ROS node
    rospy.init_node('panda_trajectory_publisher')
    contoller_name='/position_joint_trajectory_controller/command'

    # Create a publisher for the joint trajectory
    trajectory_publihser = rospy.Publisher(contoller_name,JointTrajectory, queue_size=10)

    # Load the panda robot model
    panda = rtb.models.URDF.Panda()
    argv = sys.argv[1:] 

    # Joint names                        
    panda_joints = ['panda_joint1','panda_joint2','panda_joint3','panda_joint4','panda_joint5',
                    'panda_joint6','panda_joint7']
    
    # Get point coordinates from command-line arguments
    try:
        point = SE3(float(sys.argv[1]) , float(sys.argv[2]) , float(sys.argv[3]))
    except IndexError:
        print("Error: not enough arguments. Please specify x, y and z coordinates.")
        return
    except ValueError:
        print("Error: invalid arguments. Please make sure all arguments are numbers.")
        return

    # Calculate inverse kinematics
    joint_angles = panda.ikine_LM(point).q

    # Goal positions for the joints
    goal_positions = joint_angles
    print("angles\n:", goal_positions)


    rospy.loginfo("Wait a moment, I am on my way ! ") 
    rospy.sleep(1)

    # Create a joint trajectory message
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