#!/usr/bin/env python3

import rospy
import rosbag
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pathlib import Path


def main(topic, number_of_images):
    
    if not image_dir or not bagfile_path:
        print("Invalid path")
        return
    bag = rosbag.Bag(bagfile_path)
    bridge = CvBridge()
    count = 0
    for topic, msg in bag.read_messages(topics=topic):
        count += 1
        if count % number_of_images == 0:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imwrite(str(image_dir / f"image_{count}.png"), cv_image)
            #print(msg.header.stamp)       

if __name__ == '__main__':
    """Creates images from a rosbag file
    topic: topic to be read from the rosbag file
    number_of_images: number of images to be extracted"""

    image_dir = Path('/home/selva/catkin_ws/src/needle_placement/rosbag_images/depth_images')
    bagfile_path = Path('/home/selva/Downloads/calibration/calibration_bag/calibration.bag')
    main(topic="/k4a/rgb/image_raw", number_of_images=30)
