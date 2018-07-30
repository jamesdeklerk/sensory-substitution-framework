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


image_pub = rospy.Publisher("retinal_encoded_image",Image, queue_size=2)
counter = 0
bridge = CvBridge()
RF_map = None
sampled_pixels_map = None

# C:\Users\s211114405\Downloads\fw\new.bin

# global vars
# Load in color image in grayscale
depth_image = cv2.imread(
    # 'depth_images/depth_image_d415_Depth.png')
    # 'depth_images/25_25.png')
    'depth_images/white_640_480.jpg')


# Constants
POSITION_OF_RF_PIXEL = 0
POSITIONS_OF_SAMPLED_PIXELS = 1
X = 0
Y = 1
# For a grayscale images, the pixel value is a single number that represents the brightness of the pixel
# The most common pixel format is the byte image, where this number is stored as an 8-bit integer giving a range of possible values from 0 to 255.
GRAYSCALE_MAX_VALUE = 255


# Creates a RF map where RFs are equally spaced
def setup_RF_map(image_height, image_width, map_height, map_width):

    # row
    row_increment = (image_height * 1.0) / map_height
    current_image_row = row_increment / 2

    # column
    column_increment = (image_width * 1.0) / map_width
    current_image_column = column_increment / 2

    RF_map = []
    current_RF_map_row = 0

    # row
    while (current_image_row < image_height):
        # make actual pixel, i.e. no floats
        row_chosen = math.ceil(current_image_row) - 1

        # new row array
        RF_map.append([])

        # column
        while (current_image_column < image_width):
            # make actual pixel, i.e. no floats
            column_chosen = math.ceil(current_image_column) - 1

            # create the actual RF map
            # NOTE: the RF map points are (x, y), NOT (row, column)
            RF_map[current_RF_map_row].append(
                (int(column_chosen), int(row_chosen)))

            # next x pixel
            current_image_column = current_image_column + column_increment

        # go to next row
        current_image_row = current_image_row + row_increment
        # reset starting column
        current_image_column = column_increment / 2
        current_RF_map_row = current_RF_map_row + 1

    # return RF_map - which is RF pixel positions
    # Example for 3x3 RF_map: [[(0,0),(0,1),(0,2)],[(1,0),(1,1),(1,2)],[(2,0),(2,1),(2,2)]]
    #   RF_map[0][2] = (0,2) which is the pixel position on the original image for the top right pixel of the retinal_encoded_image
    return RF_map


def draw_RF_map(image, RF_map, circle_radius, line_width, color):
    rows = len(RF_map)
    columns = len(RF_map[0])
    for row in xrange(rows):
        for column in xrange(columns):
            cv2.circle(image, RF_map[row][column],
                       circle_radius, color, line_width)
            cv2.imshow('image', image)


def sample_pixel_2d_norm_dist(RF, standard_deviation, times_to_try, image_height, image_width):
    """ Generate (x, y) by sampling a 2D normal distribution
        https://docs.scipy.org/doc/numpy-1.14.0/reference/generated/numpy.random.normal.html

        Keyword arguments:
        RF 
    """

    # get x sample
    norm_dist_sample_x = np.random.normal(RF[X], standard_deviation)
    times_tried = 1
    while True:
        # if x is in bounds
        if (norm_dist_sample_x >= 0) and (norm_dist_sample_x < image_width):
            break
        else:
            # x is out of bounds, so generate new sample
            norm_dist_sample_x = np.random.normal(RF[X], standard_deviation)
            if times_tried > times_to_try:
                # if x is out of bounds to the left set it to 0
                if norm_dist_sample_x < 0:
                    norm_dist_sample_x = 0
                # else it is out of bounds to the right, so set it to (image_width - 1)
                else:
                    norm_dist_sample_x = image_width - 1

        times_tried = times_tried + 1

    # get y sample
    norm_dist_sample_y = np.random.normal(RF[Y], standard_deviation)
    times_tried = 1
    while True:
        # if y is in bounds
        if (norm_dist_sample_y >= 0) and (norm_dist_sample_y < image_height):
            break
        else:
            # y is out of bounds, so generate new sample
            norm_dist_sample_y = np.random.normal(RF[Y], standard_deviation)
            if times_tried > times_to_try:
                # if y is out of bounds to the top set it to 0
                if norm_dist_sample_y < 0:
                    norm_dist_sample_y = 0
                # else it is out of bounds to the bottom, so set it to (image_height - 1)
                else:
                    norm_dist_sample_y = image_height - 1

        times_tried = times_tried + 1

    return (int(norm_dist_sample_x), int(norm_dist_sample_y))

def setup_sampled_pixels_map(image_height, image_width, RF_map, num_samples_per_RF):

    RF_map_rows = len(RF_map)
    RF_map_columns = len(RF_map[0])

    standard_deviation = 20
    times_to_try = 10

    sampled_pixels_map = []

    for row in xrange(RF_map_rows):
        # add a new row
        sampled_pixels_map.append([])
        for column in xrange(RF_map_columns):
            sampled_pixels_map[row].append([])
            for sample in xrange(num_samples_per_RF):
                # sample from 2D normal dist, but make sure it's within the bounds
                sampled_pixels_map[row][column].append(
                    sample_pixel_2d_norm_dist(
                        RF_map[row][column], standard_deviation,
                        times_to_try, image_height, image_width
                    )
                )

    # RF_map is an array of RFs (receptive fields) and their corrisponding sampled pixels
    #       Example:
    # single RF_map_with_positions_of_sampled_pixels = ((x, y),[(x1,y1), (x2,y2), ...])
    #                       = (position of RF pixel, array of position of sampled pixels)
    # RF_image is the image generated at the end

    # return 3D array - array[row][column] = [(x1,y1), (x2,y2), ...] (i.e. array of sampled pixels)
    return sampled_pixels_map  # sampled_pixels_map

def draw_sampled_pixels_map(image, sampled_pixels_map, RF_map, circle_radius, line_width, circle_color, line_color):
    rows = len(sampled_pixels_map)
    columns = len(sampled_pixels_map[0])
    num_samples_per_RF = len(sampled_pixels_map[0][0])
    for row in xrange(rows):
        for column in xrange(columns):
            for sample in xrange(num_samples_per_RF):
                # draw line for each sample
                cv2.line(image, sampled_pixels_map[row][column][sample], RF_map[row][column], line_color, 1)

                # draw circle for each sample
                cv2.circle(image, sampled_pixels_map[row][column][sample],
                        circle_radius, circle_color, line_width)

    # cv2.imshow('image', image)
    return image

# The activity act_i of the neuron i (a set of p pixels with luminance l_ik for the receptive field RF_i),
# is normalized into the interval [0, 1] using the following function:
#           act_i = (1/(255 * p))*sum   # where sum the sum from 1 to p of the pixel luminance values
# TODO: convert output to be in [0, 1] 
def calcNeuronActivity(positions_of_sampled_pixels, image):

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

    # image -> output image
    # 640 x 480 -> 9 x 9
    depth_image_width = len(depth_image[0])
    depth_image_height = len(depth_image)
    output_image_width = 9
    output_image_height = 9

    # setup sampled_pixels_map if it wasn't setup
    global RF_map
    global sampled_pixels_map
    if sampled_pixels_map == None:
        RF_map = setup_RF_map(depth_image_height, depth_image_width, output_image_height, output_image_width)
        num_samples_per_RF = 10
        sampled_pixels_map = setup_sampled_pixels_map(depth_image_height, depth_image_width, RF_map, num_samples_per_RF)
    
    output_image = run_melosee(depth_image, sampled_pixels_map)
    
    image_pub.publish(bridge.cv2_to_imgmsg(output_image, "32FC1"))

def retinalEncoder():
    rospy.init_node('melosee_retinal_encoder', anonymous=True)
    print('NODE RUNNING: melosee_retinal_encoder')

    # Get depth image from depth camera
    rospy.Subscriber("camera/depth/image", Image, depthCallback)

    rospy.spin()


if __name__ == '__main__':
    retinalEncoder()
