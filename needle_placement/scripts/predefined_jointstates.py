#!/usr/bin/env python3

'''Author: Selvakumar Nachimuthu
   This node publishes the pre-defined jointstates over the topic so that it plans the quintic trajectory
'''
import rospy
from sensor_msgs.msg import JointState
import numpy as np

def publish_joint_states(joint_states):
    rospy.init_node('goal_states_publisher', anonymous=True)
    pub = rospy.Publisher('/goal_states', JointState, queue_size=1)

    while pub.get_num_connections() == 0: #wait for subscriber to connect then start publishing
        rospy.loginfo("Waiting for subscribers to connect...")
        rospy.sleep(1)

    for jointangles in joint_states:
        msg = JointState()
        msg.position = jointangles
        pub.publish(msg)
        rospy.sleep(1) 
    #For taking pictures/pcds, we only need to publish once
    rospy.signal_shutdown('Published all joint states so shutting down the node')

if __name__ == '__main__':
    joint_states = [[-0.06232789538146821, -0.24622950012432895, -0.030737760180705447, -1.9726147070265652, -0.1695098137369048, 0.2420488188014485, -0.8457160456263355],
                    [-0.1011972613429507, -0.7858578941657258, -0.30894107930045506, -2.3776185930820932, 1.76484644564151, 2.975385995252124, 0.7291560173746612]]
    while not rospy.is_shutdown():
        publish_joint_states(joint_states)

