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

# Depth camera params
depth_camera_params = rospy.get_param("/depth_camera")
depth_camera_min_depth = depth_camera_params["min_depth"]
depth_camera_max_depth = depth_camera_params["max_depth"]

# Algorithm specific globals
re_algorithm_params = rospy.get_param(algorithm_name + "/re")
retinal_encoded_image_width = re_algorithm_params["output"]["retinal_encoded_image"]["width"]
retinal_encoded_image_height = re_algorithm_params["output"]["retinal_encoded_image"]["height"]
num_temporal_filter_frames = re_algorithm_params["num_temporal_filter_frames"]
num_quantization_levels = re_algorithm_params["num_quantization_levels"]

# Other globals
bridge = CvBridge()
temporal_filter_frames = deque([])
quantization_levels = None
RF_map = None
sampled_pixels_map = None
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


def run_melosee(depth_image, sampled_pixels_map):
    DEFAULT_FAR_DEPTH = 100.0  # in meters

    output_image_width = len(sampled_pixels_map[0]) # columns
    output_image_height = len(sampled_pixels_map) # rows
    num_samples_per_RF = len(sampled_pixels_map[0][0])

    output_image_array = []

    for row in xrange(output_image_height):
        # add new row
        output_image_array.append([])
        for column in xrange(output_image_width):

            total = 0.0
            count = 0.0

            # calc average 
            for sample in xrange(num_samples_per_RF):
                cur_sample = sampled_pixels_map[row][column][sample]
                x = cur_sample[0]
                y = cur_sample[1]
                cur_sample_depth = depth_image[y][x]
                if not np.isnan(cur_sample_depth):
                    total = total + cur_sample_depth
                    count = count + 1.0

            RF_depth = 0
            if not count == 0:
                RF_depth = total / count
            else:
                RF_depth = DEFAULT_FAR_DEPTH

            # add depth info to appropriate column
            output_image_array[row].append(RF_depth)

    # Create image from the image array
    # output_image_array = [[depth_image[30][30],depth_image[30][610]],[depth_image[450][30],depth_image[450][610]]]
    output_image = np.array(output_image_array, dtype=np.float32)
    
    return output_image


def retinal_encoder_algorithm(depth_image):

    depth_image_width = len(depth_image[0])
    depth_image_height = len(depth_image)

    # setup sampled_pixels_map if it wasn't setup
    global RF_map
    global sampled_pixels_map
    if sampled_pixels_map == None:
        RF_map = ssf_core.setup_RF_map(depth_image_height,
                                       depth_image_width,
                                       retinal_encoded_image_height,
                                       retinal_encoded_image_width)
        num_samples_per_RF = 10
        sampled_pixels_map = ssf_core.setup_sampled_pixels_map(depth_image_height,
                                                               depth_image_width,
                                                               RF_map,
                                                               num_samples_per_RF)

    # Apply temporal_filter
    depth_image = temporal_filter(depth_image, num_temporal_filter_frames)

    # depth_image = run_melosee(depth_image, sampled_pixels_map)

    # Quantize the image
    # TODO: When creating the quantization_levels, mimic how human
    #       depth perception attenuates
    #       OR
    #       Mimic how Auditory Depth attenuation
    quantized_depth_image = ssf_core.quantize(depth_image, quantization_levels)

    step_size_row = depth_image_height / (retinal_encoded_image_height * 1.0)
    step_size_column = depth_image_width / (retinal_encoded_image_width * 1.0)

    # NOTE: Uncomment to draw column sections on image
    # for column in xrange(retinal_encoded_image_width):
    #     for row in xrange(depth_image_height):

    #         start_col_pixel = int(math.floor(column * step_size_column))
    #         quantized_depth_image[row][start_col_pixel] = 0

    #         end_col_pixel = int(math.floor((column * step_size_column) + (step_size_column - 1)))
    #         quantized_depth_image[row][end_col_pixel] = 100

    # NOTE: Uncomment to draw row sections on image
    # for column in xrange(depth_image_width):
    #     for row in xrange(retinal_encoded_image_height):

    #         start_row_pixel = int(math.floor(row * step_size_row))
    #         quantized_depth_image[start_row_pixel][column] = 0

    #         end_row_pixel = int(math.floor((row * step_size_row) + (step_size_row - 1)))
    #         quantized_depth_image[end_row_pixel][column] = 100

    generated_image = np.zeros((retinal_encoded_image_height, retinal_encoded_image_width), dtype=np.float32)
    for column in xrange(retinal_encoded_image_width):
        for row in xrange(retinal_encoded_image_height):

            start_col_pixel = int(math.floor(column * step_size_column))
            end_col_pixel = int(math.floor((column * step_size_column) + (step_size_column - 1)))
            
            start_row_pixel = int(math.floor(row * step_size_row))
            end_row_pixel = int(math.floor((row * step_size_row) + (step_size_row - 1)))
            
            ndarray_subsection = ssf_core.crop_ndarray(quantized_depth_image,
                                                       start_col_pixel,
                                                       end_col_pixel,
                                                       start_row_pixel,
                                                       end_row_pixel)
            generated_image[row][column] = ssf_core.min_in_ndarray(ndarray_subsection)

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

    global quantization_levels
    quantization_levels = ssf_core.generate_quantization_levels(num_quantization_levels,
                                                                depth_camera_min_depth,
                                                                depth_camera_max_depth)

    # Get depth image from depth camera
    rospy.Subscriber(depth_image_topic, Image, depth_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
