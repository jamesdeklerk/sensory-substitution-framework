#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import dynamic_reconfigure.client
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError


retinal_encoded_image_pub = rospy.Publisher("retinal_encoded_image", Image, queue_size=2)
bridge = CvBridge()


# CONFIG - make into config file setting
depth_image_topic = "camera/depth/image_rect_raw"


# ------------------------------------------------------------------------------------
# TODO: Replace with your algorithm
def template_retinal_encoder_algorithm(depth_image_cv2_format):
    depth_image_width = len(depth_image_cv2_format[0])
    depth_image_height = len(depth_image_cv2_format)

    depth_image_scaled_width = 96  # depth_image_scaled_height is calculated based on the original image ratio
    depth_image_ratio = depth_image_width / (depth_image_height * 1.0)
    depth_image_scaled_height = int(depth_image_scaled_width / depth_image_ratio)

    # Resize depth image
    depth_image_scaled = cv2.resize(depth_image_cv2_format, (depth_image_scaled_width, depth_image_scaled_height))

    return depth_image_scaled
# ------------------------------------------------------------------------------------


def depth_callback(depth_image_imgmsg_format):
    # convert from ROS Image message to OpenCV Image
    # NOTE: An OpenCV Image is essentially a numpy array (super cool)
    depth_image_cv2_format = bridge.imgmsg_to_cv2(depth_image_imgmsg_format, desired_encoding="32FC1")

    # Standardised depth image units (to meters)
    depth_image_cv2_format = depth_image_cv2_format / (1000 * 1.0)
    # --------------------------------------------------------------------------------
    # TODO: Replace with your algorithm
    retinal_encoded_image = template_retinal_encoder_algorithm(depth_image_cv2_format)
    # --------------------------------------------------------------------------------

    # convert OpenCV Image to ROS Image message
    retinal_encoded_image_imgmsg_format = bridge.cv2_to_imgmsg(retinal_encoded_image, "32FC1")
    # publish
    retinal_encoded_image_pub.publish(retinal_encoded_image_imgmsg_format)


# TODO(for James): Maybe make part of ssf_core?
def retinal_encoder_algorithm_changed():
    # boot up <algorithm_name>_retinal_encoder.py
    pass


def parameter_changed_callback(config):
    
    # TODO(for James): check if retinal_encoder_algorithm changed,
    # if current file name not equal to <algorithm_name>_retinal_encoder.py
    # call retinal_encoder_algorithm_changed and pass it the new algorithm name
    rospy.loginfo(
        "Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))


def main():
    rospy.init_node("template_retinal_encoder")

    # Connect to dynamic_reconfigure server
    dynamic_reconfigure.client.Client("dynamic_parameters_server",
                                      timeout=300, config_callback=parameter_changed_callback)

    # Get depth image from depth camera
    rospy.Subscriber(depth_image_topic, Image, depth_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
