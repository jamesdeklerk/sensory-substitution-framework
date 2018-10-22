#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import dynamic_reconfigure.client
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
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

# Algorithm specific globals
re_algorithm_params = rospy.get_param(algorithm_name + "/re")
num_temporal_filter_frames = re_algorithm_params["num_temporal_filter_frames"]

# Other globals
bridge = CvBridge()
temporal_filter_frames = deque([])
# ________________________________________________________________


def temporal_filter(depth_image, num_frames):
    global temporal_filter_frames

    num_frames = 3
    if len(temporal_filter_frames) != num_frames:
        temporal_filter_frames.append(depth_image.copy())
    else:  # ready to apply the temporal filter
        current_frame = depth_image.copy()
        
        depth_image = ssf_core.temporal_filter(depth_image, temporal_filter_frames)

        # Update the frames
        temporal_filter_frames.popleft()
        temporal_filter_frames.append(current_frame)

    return depth_image


def retinal_encoder_algorithm(depth_image):

    depth_image = temporal_filter(depth_image, num_temporal_filter_frames)

    # Quantize the image
    # TODO: When creating the quantization_levels, mimic how human
    #       depth perception attenuates
    #       OR
    #       Mimic how Auditory Depth attenuation
    quantization_levels = [0.2, 0.35, 0.5, 0.65, 0.8, 1.0, 1.4, 1.8, 2.3, 2.9, 3.6, 4, 5]
    quantized_depth_image = ssf_core.quantize(depth_image, quantization_levels)

    # Get min (i.e. closest) from cluster of pixels
    depth_image_width = len(quantized_depth_image[0])
    depth_image_height = len(quantized_depth_image)

    generated_image_width = 8
    generated_image_height = 8

    step_size_row = depth_image_height / (generated_image_height * 1.0)
    step_size_column = depth_image_width / (generated_image_width * 1.0)

    # NOTE: Uncomment to draw column sections on image
    # for column in xrange(generated_image_width):
    #     for row in xrange(depth_image_height):

    #         start_col_pixel = int(math.floor(column * step_size_column))
    #         quantized_depth_image[row][start_col_pixel] = 0

    #         end_col_pixel = int(math.floor((column * step_size_column) + (step_size_column - 1)))
    #         quantized_depth_image[row][end_col_pixel] = 100

    # NOTE: Uncomment to draw row sections on image
    # for column in xrange(depth_image_width):
    #     for row in xrange(generated_image_height):

    #         start_row_pixel = int(math.floor(row * step_size_row))
    #         quantized_depth_image[start_row_pixel][column] = 0

    #         end_row_pixel = int(math.floor((row * step_size_row) + (step_size_row - 1)))
    #         quantized_depth_image[end_row_pixel][column] = 100

    generated_image = np.zeros((generated_image_height, generated_image_width), dtype=np.float32)
    for column in xrange(generated_image_width):
        for row in xrange(generated_image_height):

            start_col_pixel = int(math.floor(column * step_size_column))
            end_col_pixel = int(math.floor((column * step_size_column) + (step_size_column - 1)))
            
            start_row_pixel = int(math.floor(row * step_size_row))
            end_row_pixel = int(math.floor((row * step_size_row) + (step_size_row - 1)))
            
            ndarray_subsection = ssf_core.crop_ndarray(quantized_depth_image,
                                                       start_col_pixel,
                                                       end_col_pixel,
                                                       start_row_pixel,
                                                       end_row_pixel)
            generated_image[row][column] = ssf_core.mode_of_ndarray(ndarray_subsection)

    return generated_image


def depth_callback(depth_image_imgmsg_format):
    # convert from ROS Image message to OpenCV Image
    # NOTE: An OpenCV Image is essentially a numpy array (super cool)
    depth_image_cv2_format = bridge.imgmsg_to_cv2(depth_image_imgmsg_format, desired_encoding="32FC1")

    retinal_encoded_image = retinal_encoder_algorithm(depth_image_cv2_format)

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
