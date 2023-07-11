#!/usr/bin/env python3

"""
Author: David Sosa Gomez, Manav Thakkar and Selva Nachimuthu

This is the main script that communicates with the other nodes and publishes the angles to the robot.
"""

import rospy
from std_msgs.msg import Float64MultiArray, Bool
from sensor_msgs.msg import JointState
import numpy as np
from quintic_functions import calculateSmoothRobotTrajectory
from queue import Queue
import time

class MessageQueue:
    def __init__(self):
        self.queue = Queue()

    def enqueue(self, item):
        self.queue.put(item)

    def dequeue(self):
        if not self.queue.empty():
            return self.queue.get()
        else:
            return None

    def get_length(self):
        return self.queue.qsize()

class TrajectoryPublisher:
    def __init__(self):
        self.current_joint_states = None
        self.desired_joint_states = None
        self.interpolated_joint_states = None
        self.goal_message_queue = MessageQueue() #stores messages over goal topic
        self.interpolated_message_queue = MessageQueue()  #stores messages over interpolated_angles topic

        rospy.init_node('quintic_trajectory_publisher_sim', anonymous=True)

        #publishers
        self.pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)
        self.reached_pub = rospy.Publisher('/reached', Bool, queue_size=1)
        self.trajectory_done_pub = rospy.Publisher('/trajectory_done', Bool, queue_size=1)
        self.is_trajectory_done = False

        #subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/goal_states', JointState, self.desired_goal_callback)   #goal_states subscriber
        rospy.Subscriber('/interpolated_angles', JointState, self.interpolation_callback)  #interpolated_angles subscriber 

    def joint_states_callback(self, msg):
        self.current_joint_states = msg.position

    def interpolation_callback(self, msg):
        self.interpolated_message_queue.enqueue(msg)
        self.interpolated_joint_states = msg.position
        print("Message queue length (interpolated):", self.interpolated_message_queue.get_length())

    def desired_goal_callback(self, msg):
        self.goal_message_queue.enqueue(msg)  #pushing it to queue
        self.desired_joint_states = msg.position
        print("Message queue length (goal state):", self.goal_message_queue.get_length())

    # Publish the trajectory after waiting for current and desired joint states to be available
    def publish_trajectory(self):
        while self.current_joint_states is None or self.desired_joint_states is None:
            rospy.sleep(0.1)
        next_message = self.goal_message_queue.dequeue()
        if next_message is not None:
            self.calculate_trajectory(next_message)

    # Publish interpolated trajectory if both current joint states and interpolated joint states are available
    def publish_interpolated_trajectory(self):
        while self.current_joint_states is None or self.interpolated_joint_states is None:   
            rospy.sleep(0.1) 

        concatenated_trajectory = np.empty((0, len(self.current_joint_states))) 
        
        # Continue dequeuing and processing interpolated trajectory points until queue is empty
        while self.interpolated_message_queue.get_length() > 0:  
            next_message = self.interpolated_message_queue.dequeue()
            print('remaining messages in interpolated queue:', self.interpolated_message_queue.get_length())
            if next_message is not None:
                trajectory = calculateSmoothRobotTrajectory(self.current_joint_states, next_message.position, False)
                if trajectory is None:
                    continue
                concatenated_trajectory = np.vstack((concatenated_trajectory, trajectory)) 
                self.current_joint_states = next_message.position

        rate = rospy.Rate(1000) 
        for i in range(concatenated_trajectory.shape[0]):
            trajectory_point = concatenated_trajectory[i] 
            self.current_joint_states = trajectory_point
            msg = Float64MultiArray()
            msg.data = trajectory_point.tolist()
            self.pub.publish(msg)
            rate.sleep()


    def calculate_trajectory(self, msg):
        self.desired_joint_states = msg.position
        trajectories = calculateSmoothRobotTrajectory(self.current_joint_states, self.desired_joint_states, False)
        if trajectories is None:
            return 
        print("Trajectory calculated.....publishing....")

        rate = rospy.Rate(1000)
        for trajectory_point in trajectories:
            msg = Float64MultiArray()
            msg.data = trajectory_point.tolist()
            self.pub.publish(msg)
            rate.sleep()

        print("Pose reached")
        time.sleep(2)
        self.is_trajectory_done = True
        print('Trajectory completed')

        # Publish True to /reached topic to indicate that the needle is at entry point
        reached_msg = Bool()
        reached_msg.data = True
        self.reached_pub.publish(reached_msg)

    def run(self):
        while not rospy.is_shutdown():
            if self.is_trajectory_done:
                trajectory_done_msg = Bool()
                trajectory_done_msg.data = True
                self.trajectory_done_pub.publish(trajectory_done_msg)
                rospy.sleep(1)
                self.is_trajectory_done = False
            if self.goal_message_queue.get_length() > 0:
                self.publish_trajectory()
            elif self.interpolated_message_queue.get_length() > 0 and self.goal_message_queue.get_length() == 0: 
                self.publish_interpolated_trajectory()
            else:
                rospy.sleep(0.1)
                
if __name__ == '__main__':
    trajectory_publisher = TrajectoryPublisher()
    trajectory_publisher.run()
