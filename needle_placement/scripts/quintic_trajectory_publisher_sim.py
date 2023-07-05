#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
from quintic_functions import calculateSmoothRobotTrajectory
from queue import Queue

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
        self.message_queue = MessageQueue()

        rospy.init_node('quintic_trajectory_publisher_sim', anonymous=True)
        self.pub = rospy.Publisher('/joint_position_example_controller_sim/joint_command', Float64MultiArray, queue_size=10)
        #self.trajectordone_pub = rospy.Publisher('/trajectory_done', Bool, queue_size=1)
        # Subscribe to joint states topic
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # Subscribe to desired goal topic
        rospy.Subscriber('/goal_states', JointState, self.desired_goal_callback)

        rospy.Subscriber('/interpolated_angles', JointState, self.interpolation_callback)
        

    def joint_states_callback(self, msg):
        self.current_joint_states = msg.position

    def interpolation_callback(self, msg):
        self.message_queue.enqueue(msg)
        self.interpolated_joint_states = msg.position
        print("Message queue length:", self.message_queue.get_length())

    def desired_goal_callback(self, msg):
        #print('Received message')
        #push the message to the queue
        self.message_queue.enqueue(msg)
        self.desired_joint_states = msg.position
        print("Message queue length:", self.message_queue.get_length())

    def publish_trajectory(self):
        # Wait for the initial joint states and desired joint states to be received
        while self.current_joint_states is None or self.desired_joint_states is None:
            rospy.sleep(0.1)
        #if the queue is populated then it means it received a message when the trajectory is being performed
        #next message is considered as the desired_joint_states (FIFO)
        next_message = self.message_queue.dequeue()
        if next_message is not None:
            self.calculate_trajectory(next_message)

    def calculate_trajectory(self, msg):
        # Extract the desired joint states from the message
        self.desired_joint_states = msg.position

        # Calculate trajectory
        trajectories = calculateSmoothRobotTrajectory(self.current_joint_states, self.desired_joint_states, False)

        if trajectories is None:
            return 
        print("Trajectory calculated.....publishing....")

        # Loop through the trajectory and publish each row
        rate = rospy.Rate(1000)  # Hz
        for trajectory_point in trajectories:
            msg = Float64MultiArray()
            msg.data = trajectory_point.tolist()
            self.pub.publish(msg)
            rate.sleep()

        #trajectory_done_msg = Bool()
        #trajectory_done_msg.data = True
        #self.trajectorydone_pub.publish(trajectory_done_msg)

    def run(self):
        try:
            while not rospy.is_shutdown():
                self.publish_trajectory()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    trajectory_publisher = TrajectoryPublisher()
    trajectory_publisher.run()
