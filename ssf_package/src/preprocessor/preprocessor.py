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

# https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
INTERPOLATION_DICT = {
    "INTER_NEAREST": cv2.INTER_NEAREST,
    "INTER_LINEAR": cv2.INTER_LINEAR, # cv2 default
    "INTER_AREA": cv2.INTER_AREA,
    "INTER_CUBIC": cv2.INTER_CUBIC,
    "INTER_LANCZOS4": cv2.INTER_LANCZOS4
}

# CONFIG
interpolation_used = "INTER_LINEAR"
depth_image_scaled_width = 96 # depth_image_scaled_height is calculated based on the original image ratio

def depthCallback(depth_data):

    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="32FC1")

    depth_image_width = len(depth_image[0])
    depth_image_height = len(depth_image)

    depth_image_ratio = depth_image_width / (depth_image_height * 1.0)
    depth_image_scaled_height = int(depth_image_scaled_width / depth_image_ratio)

    # INTER_LINEAR is used by default if no interpolation is specified
    depth_image_scaled = cv2.resize(depth_image, (depth_image_scaled_width, depth_image_scaled_height), interpolation=INTERPOLATION_DICT[interpolation_used])
    depth_image_scaled = depth_image_scaled / 1000.0

    # Create image from the image array
    # output_image_array = [[depth_image[30][30],depth_image[30][610]],[depth_image[450][30],depth_image[450][610]]]
    # output_image = np.array(depth_image, dtype=np.float32)
    processed_depth_image_pub.publish(bridge.cv2_to_imgmsg(depth_image_scaled, "32FC1"))


def preprocessor():
    rospy.init_node('preprocessor', anonymous=True)
    print('NODE RUNNING: preprocessor')

    # Get depth image from depth camera
    rospy.Subscriber("camera/depth/image_rect_raw", Image, depthCallback)

    rospy.spin()

if __name__ == '__main__':
    preprocessor()