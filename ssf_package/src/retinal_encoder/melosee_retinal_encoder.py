#!/usr/bin/env python

# pip install opencv-python
# pip install numpy
# pip install matplotlib

import math
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# importing the ssf_core module
import rospkg
from sys import path as system_path
from os import path as os_path
rospack = rospkg.RosPack()
core_package_path = os_path.join(rospack.get_path('ssf_package'), 'src', 'core')
system_path.append(core_package_path)
import ssf_core


image_pub = rospy.Publisher("retinal_encoded_image",Image, queue_size=2)
counter = 0
bridge = CvBridge()
RF_map = None
sampled_pixels_map = None

# C:\Users\s211114405\Downloads\fw\new.bin


# Depth camera params
depth_camera_params = rospy.get_param("/depth_camera")
depth_value_divisor = depth_camera_params["depth_value_divisor"]


# The activity act_i of the neuron i (a set of p pixels with luminance l_ik for the receptive field RF_i),
# is normalized into the interval [0, 1] using the following function:
#           act_i = (1/(255 * p))*sum   # where sum the sum from 1 to p of the pixel luminance values
# TODO: convert output to be in [0, 1] 
def calcNeuronActivity(positions_of_sampled_pixels, image):
    X = 0
    Y = 1

    # Example input:
    #   RF_neighboring_pixels = [(1,2), (3,4), (5,6)]

    number_of_pixels = len(positions_of_sampled_pixels)

    # Summing luminance of p sampled pixels
    sum = 0
    for k in xrange(number_of_pixels):
        current_pixel_position = positions_of_sampled_pixels[k]
        # summing current pixels luminance
        sum = sum + image(current_pixel_position[X], current_pixel_position[Y])

    # NOTE:
    # The algorithm returns the normalized neuron activity, I don't think I should normalize,
    # because in ROS you get the meter value, not a 0-255 grayscale value, so rather just get average meter value
    # return (1 / (GRAYSCALE_MAX_VALUE * p)) * sum
    # Returns neuron activity
    return sum / number_of_pixels


# TODO: make sure youre doing the x and y correctly
def run_melosee(depth_image, sampled_pixels_map):
    DEFAULT_FAR_DEPTH = 100

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


def depthCallback(depth_data):
    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="32FC1")

    # log out the distance to a specific point
    # rospy.loginfo('Distance at 30x, 30y pixel: {}m'.format(depth_image[30][30]))

    # depth image -> output image
    # original width x original height -> 8 x 8
    depth_image_width = len(depth_image[0])
    depth_image_height = len(depth_image)
    output_image_width = 8
    output_image_height = 8

    # setup sampled_pixels_map if it wasn't setup
    global RF_map
    global sampled_pixels_map
    if sampled_pixels_map == None:
        RF_map = ssf_core.setup_RF_map(depth_image_height, depth_image_width, output_image_height, output_image_width)
        num_samples_per_RF = 10
        sampled_pixels_map = ssf_core.setup_sampled_pixels_map(depth_image_height, depth_image_width, RF_map, num_samples_per_RF)
    
    # Since this uses the raw camera image (camera/depth/image_rect_raw).
    # This step converts the depth image pixel values to meters,
    # since meters is the standard used in this framework 
    depth_image = depth_image / depth_value_divisor
    
    output_image = run_melosee(depth_image, sampled_pixels_map)
    # quantization_levels = [0.2, 0.35, 0.5, 0.65, 0.8, 1.0, 1.4, 1.8, 2.3, 2.9, 3.6, 4, 5]
    # output_image = ssf_core.quantize(output_image, quantization_levels)
    # depth_image = depth_image / 1000.0
    # depth_image = ssf_core.draw_RF_map(depth_image,
    #                                    RF_map,
    #                                    2,
    #                                    2,
    #                                    (0, 0, 0))
    # output_image = ssf_core.draw_sampled_pixels_map(depth_image,
    #                                                 sampled_pixels_map,
    #                                                 RF_map,
    #                                                 1,
    #                                                 2,
    #                                                 (0, 0, 0),
    #                                                 (0, 0, 0))
    
    image_pub.publish(bridge.cv2_to_imgmsg(output_image, "32FC1"))


def main():
    rospy.init_node('melosee_retinal_encoder')
    print('NODE RUNNING: melosee_retinal_encoder')

    # Get depth image from depth camera
    rospy.Subscriber("camera/depth/image_rect_raw", Image, depthCallback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
