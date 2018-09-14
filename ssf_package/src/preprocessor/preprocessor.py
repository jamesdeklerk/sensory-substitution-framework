#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


processed_depth_image_pub = rospy.Publisher("processed_depth_image",Image, queue_size=2)
bridge = CvBridge()


def depthCallback(depth_data):
    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="32FC1")

    # log out the distance to a specific point
    rospy.loginfo('Distance at 30x, 30y pixel: {}m'.format(depth_image[30][30]))

    # Create image from the image array
    # output_image_array = [[depth_image[30][30],depth_image[30][610]],[depth_image[450][30],depth_image[450][610]]]
    output_image = np.array(depth_image, dtype=np.float32)
    processed_depth_image_pub.publish(bridge.cv2_to_imgmsg(output_image, "32FC1"))


def preprocessor():
    rospy.init_node('preprocessor', anonymous=True)
    print('NODE RUNNING: preprocessor')

    # Get depth image from depth camera
    rospy.Subscriber("camera/depth/image_rect_raw", Image, depthCallback)

    rospy.spin()

if __name__ == '__main__':
    preprocessor()