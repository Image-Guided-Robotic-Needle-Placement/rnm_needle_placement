#!/usr/bin/env python3

'''
It saves all the info to the rosbag (saves data for 1 second)
To send true to the topic --> rostopic pub /goal_reached std_msgs/Bool "data: true"
saves the rgb and ir images
minor issues with the storing of rosbag files 
'''
import rospy
import rosbag
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from rospy import Duration
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import cv2
import os

class BagRecorderNode:
    def __init__(self):
        self.reached_position = False
        self.bridge = CvBridge()
        self.rgbimage_path = "../src/needle_placement/new/rgb_images/"
        self.irimage_path = "../src/needle_placement/new/ir_images/"

        self.subscriber = rospy.Subscriber('/goal_reached', Bool, self.reached_position_callback)

    def reached_position_callback(self, data):
        if data.data:
            rospy.loginfo("goal reached")
  
            rgb_msg = rospy.wait_for_message("/k4a/rgb/image_raw", Image, timeout=rospy.Duration(1))
            ir_msg = rospy.wait_for_message("/k4a/ir/image_raw", Image, timeout=rospy.Duration(1))
            joint_msg = rospy.wait_for_message('/franka_state_controller/joint_states_desired', JointState, timeout=rospy.Duration(1))
            
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            ir_image = self.bridge.imgmsg_to_cv2(ir_msg, desired_encoding="bgr8")

        timestamp = rospy.Time.now()
        rgb_filename = self.rgbimage_path + "rgb_image_" + str(timestamp) + ".png"
        ir_filename = self.irimage_path + "ir_image_" + str(timestamp) + ".png"

        cv2.imwrite(rgb_filename, rgb_image)
        cv2.imwrite(ir_filename, ir_image)
        print("image saved")

        rospy.loginfo("Bag recording completed!")
        rospy.signal_shutdown("Recording and image extraction completed")

if __name__ == "__main__":
    rospy.init_node("bag_recorder_node")
    node = BagRecorderNode()
    rospy.spin()
