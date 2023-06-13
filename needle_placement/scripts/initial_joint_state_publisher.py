#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def publish_joint_states():
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz

    joint_state = JointState()
    joint_state.name = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
    joint_state.position = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.789]  # your desired initial joint positions
    joint_state.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        pub.publish(joint_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_joint_states()
    except rospy.ROSInterruptException:
        pass
