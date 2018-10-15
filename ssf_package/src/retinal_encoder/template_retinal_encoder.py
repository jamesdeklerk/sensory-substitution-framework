#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import dynamic_reconfigure.client
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

# importing the ssf_core module
import rospkg
from sys import path as system_path
from os import path as os_path
rospack = rospkg.RosPack()
core_package_path = os_path.join(rospack.get_path('ssf_package'), 'src', 'core')
system_path.append(core_package_path)
import ssf_core


# ----------------------------------------------------------------
#                            GLOBALS
# ----------------------------------------------------------------
# Algorithm name (from the filename)
file_name = os_path.splitext(os_path.basename(__file__))[0]
algorithm_name = file_name.replace("_retinal_encoder", "")
ssf_core.check_algorithm_name(algorithm_name)

# Input params
re_input_params = rospy.get_param("/re/input")
depth_image_topic = re_input_params["depth_image"]["topic"]

# Output params
re_output_params = rospy.get_param("/re/output")
retinal_encoded_image_pub = rospy.Publisher(re_output_params["depth_image"]["topic"],
                                            Image, queue_size=2)

# Other globals
bridge = CvBridge()

# Algorithm specific globals
re_algorithm_params = rospy.get_param(algorithm_name + "/re")
# TODO: Replace with your algorithm params
depth_image_scaled_width = re_algorithm_params["depth_image_scaled_width"]
# ________________________________________________________________


# ------------------------------------------------------------------------------------
# TODO: Replace with your algorithm
#       The default template algorithm simply scales the image down
def retinal_encoder_algorithm(depth_image_cv2_format):
    depth_image_width = len(depth_image_cv2_format[0])
    depth_image_height = len(depth_image_cv2_format)

    depth_image_ratio = depth_image_width / (depth_image_height * 1.0)
    # depth_image_scaled_height is calculated based on the original image ratio
    depth_image_scaled_height = int(depth_image_scaled_width / depth_image_ratio)

    # Resize depth image
    depth_image_scaled = cv2.resize(depth_image_cv2_format, (depth_image_scaled_width, depth_image_scaled_height))

    return depth_image_scaled
# ____________________________________________________________________________________


def depth_callback(depth_image_imgmsg_format):
    # convert from ROS Image message to OpenCV Image
    # NOTE: An OpenCV Image is essentially a numpy array (super cool)
    depth_image_cv2_format = bridge.imgmsg_to_cv2(depth_image_imgmsg_format, desired_encoding="32FC1")

    
    # NOTE: Uncomment line below if not using preprocessor
    # depth_image_cv2_format = depth_image_cv2_format / (1000 * 1.0)

    # --------------------------------------------------------------------------------
    # TODO: Replace with your algorithm
    retinal_encoded_image = retinal_encoder_algorithm(depth_image_cv2_format)
    # ________________________________________________________________________________

    # convert OpenCV Image to ROS Image message
    retinal_encoded_image_imgmsg_format = bridge.cv2_to_imgmsg(retinal_encoded_image, "32FC1")
    # publish
    retinal_encoded_image_pub.publish(retinal_encoded_image_imgmsg_format)


def parameter_changed_callback(config):
    pass


def main():
    rospy.init_node(file_name)
    print("NODE RUNNING: " + file_name)

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
