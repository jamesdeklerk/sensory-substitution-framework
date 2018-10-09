#!/usr/bin/env python


# importing the ssf_core module
import rospkg
from sys import path as system_path
from os import path as os_path
rospack = rospkg.RosPack()
core_package_path = os_path.join(rospack.get_path('ssf_package'), 'src', 'core')
system_path.append(core_package_path)
import ssf_core


import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
INTERPOLATION_DICT = {
    "INTER_NEAREST": cv2.INTER_NEAREST,
    "INTER_LINEAR": cv2.INTER_LINEAR,  # cv2 default
    "INTER_AREA": cv2.INTER_AREA,
    "INTER_CUBIC": cv2.INTER_CUBIC,
    "INTER_LANCZOS4": cv2.INTER_LANCZOS4
}


# CONFIG - make into config file setting
depth_image_topic = "camera/depth/image_rect_raw"
color_image_topic = "camera/color/image_raw"
interpolation_used = "INTER_LINEAR"
depth_image_scaled_width = 96  # depth_image_scaled_height is calculated based on the original image ratio
color_image_scaled_width = 96  # color_image_scaled_height is calculated based on the original image ratio


processed_depth_image_pub = rospy.Publisher("processed_depth_image", Image, queue_size=2)
processed_color_image_pub = rospy.Publisher("processed_color_image", Image, queue_size=2)
bridge = CvBridge()


def depth_callback(depth_data):

    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="32FC1")

    # Crop the dead zones off the image
    depth_image = ssf_core.crop_image(image=depth_image, crop_width_per=0.1, crop_height_per=0)

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


def color_callback(color_data):

    color_image = bridge.imgmsg_to_cv2(color_data, desired_encoding="bgr8")

    color_image_width = len(color_image[0])
    color_image_height = len(color_image)

    color_image_ratio = color_image_width / (color_image_height * 1.0)
    color_image_scaled_height = int(color_image_scaled_width / color_image_ratio)

    # INTER_LINEAR is used by default if no interpolation is specified
    color_image_scaled = cv2.resize(color_image, (color_image_scaled_width, color_image_scaled_height), interpolation=INTERPOLATION_DICT[interpolation_used])

    # Create image from the image array
    # output_image_array = [[color_image[30][30],color_image[30][610]],[color_image[450][30],color_image[450][610]]]
    # output_image = np.array(color_image, dtype=np.float32)
    processed_color_image_pub.publish(bridge.cv2_to_imgmsg(color_image_scaled, "bgr8"))


def main():
    rospy.init_node('preprocessor', anonymous=True)
    print('NODE RUNNING: preprocessor')
    print('color_image_scaled_width: {}px'.format(color_image_scaled_width))
    print('depth_image_scaled_width: {}px'.format(depth_image_scaled_width))

    # Get depth image from depth camera
    rospy.Subscriber(depth_image_topic, Image, depth_callback)

    # Get depth image from color camera
    rospy.Subscriber(color_image_topic, Image, color_callback)

    rospy.spin()


if __name__ == '__main__':
    main()