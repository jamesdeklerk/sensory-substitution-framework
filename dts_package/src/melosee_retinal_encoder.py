# pip install opencv-python
# pip install numpy
# pip install matplotlib

import math
import numpy as np
import cv2

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

    cv2.imshow('image', image)

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


def main():

    image_height = len(depth_image)
    image_width = len(depth_image[0])

    # setup window and display image
    cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', int(image_width * 0.8), int(image_height * 0.8))
    cv2.moveWindow('image', 0, 0)
    cv2.imshow('image', depth_image)

    # RF_map = setup_RF_map(image_height, image_width, 25, 25)
    # RF_map = setup_RF_map(image_height, image_width, 2,1)
    # draw_RF_map(depth_image, RF_map, 1, 1)

    RF_map = setup_RF_map(image_height, image_width, 8, 8)

    sampled_pixels_map = setup_sampled_pixels_map(image_height, image_width, RF_map, 10)
    draw_sampled_pixels_map(depth_image, sampled_pixels_map, RF_map, 2, 2, (0,0,255), (255,0,0))

    draw_RF_map(depth_image, RF_map, 2, 2, (0, 255, 0))

    # click Esc key (with one of the image windows in focus) to stop
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    exit()


if __name__ == '__main__':
    main()
