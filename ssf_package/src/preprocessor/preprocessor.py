#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

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
# Input params
pp_input_params = rospy.get_param("/pp/input")
depth_image_topic = pp_input_params["depth_image"]["topic"]
color_image_topic = pp_input_params["color_image"]["topic"]

# Output params
pp_output_params = rospy.get_param("/pp/output")
# depth_image_scaled_height is calculated based on the original image ratio
depth_image_scaled_width = pp_output_params["depth_image"]["width"]
# color_image_scaled_height is calculated based on the original image ratio
color_image_scaled_width = pp_output_params["color_image"]["width"]
processed_depth_image_pub = rospy.Publisher(pp_output_params["depth_image"]["topic"],
                                            Image, queue_size=2)
processed_color_image_pub = rospy.Publisher(pp_output_params["color_image"]["topic"],
                                            Image, queue_size=2)

# General params
pp_general_params = rospy.get_param("/pp/general")
interpolation_used = pp_general_params["interpolation_used"]
num_temporal_filter_frames = pp_general_params["num_temporal_filter_frames"]

# Depth camera params
depth_camera_params = rospy.get_param("/depth_camera")
crop_width_percentage = depth_camera_params["crop_width_percentage"]
crop_height_percentage = depth_camera_params["crop_height_percentage"]
depth_value_divisor = depth_camera_params["depth_value_divisor"]

# Other globals
bridge = CvBridge()
temporal_filter_frames = deque([])
# https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize
INTERPOLATION_DICT = {
    "INTER_NEAREST":    cv2.INTER_NEAREST,
    "INTER_LINEAR":     cv2.INTER_LINEAR,  # cv2 default
    "INTER_AREA":       cv2.INTER_AREA,
    "INTER_CUBIC":      cv2.INTER_CUBIC,
    "INTER_LANCZOS4":   cv2.INTER_LANCZOS4
}
# ________________________________________________________________


def temporal_filter(depth_image, num_frames):
    global temporal_filter_frames

    if len(temporal_filter_frames) != num_frames:
        temporal_filter_frames.append(depth_image.copy())
    else:  # ready to apply the temporal filter
        current_frame = depth_image.copy()
        
        depth_image = ssf_core.temporal_filter(depth_image, temporal_filter_frames)

        # Update the frames
        temporal_filter_frames.popleft()
        temporal_filter_frames.append(current_frame)

    return depth_image


def depth_callback(depth_data):

    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="32FC1")

    # Crop the dead zones off the image
    depth_image_cropped = ssf_core.crop_image(image=depth_image,
                                              crop_width_per=crop_width_percentage,
                                              crop_height_per=crop_height_percentage)

    # Apply a temporal filter
    depth_image_cropped = temporal_filter(depth_image_cropped, num_temporal_filter_frames)

    depth_image_width = len(depth_image_cropped[0])
    depth_image_height = len(depth_image_cropped)

    depth_image_ratio = depth_image_width / (depth_image_height * 1.0)
    depth_image_scaled_height = int(depth_image_scaled_width / depth_image_ratio)

    # INTER_LINEAR is used by default if no interpolation is specified
    depth_image_scaled = cv2.resize(depth_image_cropped, (depth_image_scaled_width, depth_image_scaled_height),
                                    interpolation=INTERPOLATION_DICT[interpolation_used])
    # Standardize depth image units (to meters)
    depth_image_scaled = depth_image_scaled / depth_value_divisor

    # When the image is scaled using cv2.resize, all NaN's are set to 0, hence 
    # If anything is 0, that means it was out of range (either too far
    # or too close), so we reset it to NaN
    # NOTE: If they are not set to NaN, things like ssf_core.temporal_filter
    #       won't work as expected.
    depth_image_scaled[depth_image_scaled == 0.0] = np.nan

    # Publish the processed image
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