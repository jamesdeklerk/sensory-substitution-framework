#!/usr/bin/env python

"""Sensory Substitution Framework (SSF) Core

This module contains commonly used functions for the
Sensory Substitution Framework.

"""

import rospy
import math
import warnings
import cv2
import numpy as np
from scipy import stats
from scipy.signal import blackmanharris
import scipy.io.wavfile as wav


def check_algorithm_name(algorithm_name):
    invalid_names = ["pp",
                     "re",
                     "sg",
                     "color_camera",
                     "depth_camera",
                     "dynamic_parameters_server",
                     "rosdistro",
                     "rosversion",
                     "test_1"]

    for invalid_name in invalid_names:
        if algorithm_name == invalid_name:
            print("WARNING: The following algorithm names are invalid: {},\
                  please change your algorithm name to avoid errors"
                  .format(invalid_names))


def calc_cropped_fov(h_fov,
                     v_fov,
                     crop_width_percentage,
                     crop_height_percentage):
    """Calculates the cropped FoV
    
    Returns:
        tuple: (cropped_h_fov, cropped_v_fov)
    """
    
    cropped_h_fov = h_fov * (1.0 - crop_width_percentage)
    cropped_v_fov = v_fov * (1.0 - crop_height_percentage)
    return (cropped_h_fov, cropped_v_fov)


def crop_ndarray(ndarray,
                 x_start_index,
                 x_end_index,
                 y_start_index,
                 y_end_index):
    """Crop ndarray by index

    NOTE:
    - x_end_index is inclusive of the index given
    - y_end_index is inclusive of the index given
    """
    return ndarray[y_start_index:(y_end_index + 1), x_start_index:(x_end_index + 1)]


def crop_image(image, crop_width_per=0, crop_height_per=0):
    """Crop the given image by percent

    This function crops (crop_width_per / 2.0)% from the left 
    and (crop_width_per / 2.0)% from the right of the image.
    It also crops (crop_height_per / 2.0)% from the top 
    and (crop_height_per / 2.0)% from the bottom of the image.
    """
    image_width = len(image[0])
    image_height = len(image)

    if crop_width_per != 0:
        half_crop_width_per = crop_width_per / 2.0
        num_pixels_to_crop_per_side_W = int(round(image_width * half_crop_width_per))
        
        x_start_index = 0 + num_pixels_to_crop_per_side_W
        x_end_index = (image_width - 1) - num_pixels_to_crop_per_side_W
    else:
        x_start_index = 0
        x_end_index = (image_width - 1)

    if crop_height_per != 0:
        half_crop_height_per = crop_height_per / 2.0
        num_pixels_to_crop_per_side_H = int(round(image_height * half_crop_height_per))
        
        y_start_index = 0 + num_pixels_to_crop_per_side_H
        y_end_index = (image_height - 1) - num_pixels_to_crop_per_side_H
    else:
        y_start_index = 0
        y_end_index = (image_height - 1)

    return crop_ndarray(image, x_start_index, x_end_index, y_start_index, y_end_index)


def k_means(depth_image, num_clusters, min_depth, max_depth):
    rows = len(depth_image)
    columns = len(depth_image[0])
    depth_image = depth_image.reshape((rows * columns, 1))

    depth_image[depth_image < min_depth] = min_depth
    depth_image[depth_image > max_depth] = max_depth

    depth_image[np.isnan(depth_image)] = 0.0

    # define criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    # apply kmeans()
    ret, label, k_center_values = cv2.kmeans(depth_image, num_clusters, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    res = k_center_values[label.flatten()]
    depth_image = res.reshape((rows, columns))

    depth_image[depth_image == 0.0] = np.nan

    return depth_image


def generate_quantization_levels(num_quantization_levels, min_depth, max_depth):
    """Generate quantization_levels, taking into account the number of
    quantization levels wanted, as well as the min and max depth

    NOTE: The quantization_levels are rounded to 2 decimal places
    """
    
    step_size = ((2.0 * (max_depth - min_depth)) / (num_quantization_levels * 1.0)) / (num_quantization_levels - 1.0)

    quantization_levels = []
    current_quantization_value = min_depth
    for i in xrange(num_quantization_levels):
        current_quantization_value = current_quantization_value + (i * step_size)
        quantization_levels.append(round(current_quantization_value, 2))

    return quantization_levels


def quantize(depth_image,
             quantization_levels):
    """Quantize a given depth image.

    Example:
        For these params:
            quantization_levels = [20.0, 31.4, 42.8, 54.2, 65.7, 77.1, 88.5, 100.0]
        Each depth value in the image:
            0    to <31.4   mapped to   20.0
            31.4 to <42.8   mapped to   31.4
            42.8 to <54.2   mapped to   42.8
            54.2 to <65.7   mapped to   54.2
            65.7 to <77.1   mapped to   65.7
            77.1 to <88.5   mapped to   77.1
            88.5 to <100.0  mapped to   88.8
            >=100.0         mapped to   100.0

    """

    with warnings.catch_warnings():
        # ignore all the warnings that occur when handling NaN's 
        warnings.simplefilter("ignore", category=RuntimeWarning)

        num_quantization_levels = len(quantization_levels)

        depth_image[depth_image < quantization_levels[1]] = quantization_levels[0]
        
        for i in xrange(num_quantization_levels - 2):
            depth_image[(depth_image < quantization_levels[i + 2]) &
                        (depth_image >= quantization_levels[i + 1])] = quantization_levels[i + 1]
        
        depth_image[depth_image >=
                    quantization_levels[num_quantization_levels - 1]] = quantization_levels[num_quantization_levels - 1]

    return depth_image


def min_in_ndarray(ndarray):
    """Returns the min value in the given ndarray"""
    return np.nanmin(ndarray)


# # Commented out, since it causes a strange lag in processing when there
# # are too many NaN values in the ndarray
# def mode_of_ndarray(ndarray):
#     """Calculate the mode of a given ndarray.

#     The mode is the most frequently occuring number found in a given set
#     """
#     flat_ndarray = ndarray.flatten()
#     return stats.mode(flat_ndarray)[0][0]


def temporal_filter(depth_image, filter_frames):
    """Averages the pixel values across frames

    The temporal filter reduces flickering in the depth image by averaging
    the pixel values across frames. Aditionally NaN values are not included
    in the averaging process (i.e. average of [3, NaN, 3] is 3, not 2),
    this helps reduce flickering and depth mismatches.

    Args:
        depth_image: A depth image in numpy array format
        filter_frames: a list of depth_image frames used for filtering,
            this is usually the previous +-3 frames

    Returns:
        The image with the temporal filter applied

    Usage Example:
        # At the top of the file add these two lines:
        # from collections import deque
        # _temporal_filter_frames = deque([])
        
        # Then in the frame callback function, add the rest found below
        global _temporal_filter_frames

        num_frames = 3
        if len(_temporal_filter_frames) != num_frames:
            _temporal_filter_frames.append(depth_image.copy())
        else:  # ready to apply the temporal filter
            current_frame = depth_image.copy()
            
            depth_image = ssf_core.temporal_filter(depth_image, _temporal_filter_frames)

            # Update the frames
            _temporal_filter_frames.popleft()
            _temporal_filter_frames.append(current_frame)
        
        return depth_image

    """

    # Blend the frames (i.e. take the average)
    combined_array = [depth_image]
    for frame in filter_frames:
        combined_array.append(frame)

    combined_array = np.array(combined_array)
    combined_array[combined_array == 0.0] = np.nan

    with warnings.catch_warnings():
        # ignore all the warnings that occur when handling NaN's 
        warnings.simplefilter("ignore", category=RuntimeWarning)

        mean_image = np.nanmean(combined_array, 0)

    return mean_image


# ----------------------------------------------------------------
#                    3D PROCESSING FUNCTIONS
# ----------------------------------------------------------------

def calc_pixel_size(distance_to_near_plane,
                    h_fov,
                    v_fov,
                    image_width,
                    image_height):
    """Calculates the width and height of the pixels
    NOTE: it is assumed that all pixels have the same dimensions

    Args:
        h_fov: Must be in radians
        v_fov: Must be in radians

    Returns:
        2-tuple: (pixel_width, pixel_height)
    """
    pixel_width = ((2.0 * (distance_to_near_plane * 1.0)) * math.tan(h_fov / 2.0)) / image_width
    pixel_height = ((2.0 * (distance_to_near_plane * 1.0)) * math.tan(v_fov / 2.0)) / image_height
    return (pixel_width, pixel_height)


# # For vertical
# # - pixel_size = pixel height
# # For horizontal
# # - pixel_size = pixel width
# # pixel_number = i-th pixel from the centre of the image (starting at 1)
# def calc_angle(self, distance_to_near_plane, pixel_size, pixel_number):
#     # if odd number of pixels, add half pixel width?
#     return math.atan(((pixel_size * pixel_number * 1.0) - (pixel_size / 2.0)) / (distance_to_near_plane * 1.0))


# # Calculate projected position for x or y
# # depth is depth along ray
# def calc_pos_x_or_y(depth, angle):
#     return depth * math.sin(angle)


# # Always calculated according to the x-axis
# # depth is depth along ray
# def calc_pos_z(depth, angle):
#     return depth * math.cos(angle)


def calc_unit_vector(pixel_width,
                     pixel_height,
                     x_th_pixel_from_centre,
                     y_th_pixel_from_centre,
                     num_x_pixels,
                     num_y_pixels,
                     distance_to_near_plane):
    """Calculate the unit vector for the given params

    Returns:
        3-tuple: The unit vector e.g. (x, y, z)
    """
    
    # Calculate x position
    image_width_odd = not ((num_x_pixels % 2.0) == 0.0)
    if image_width_odd:
        x_position = (x_th_pixel_from_centre * pixel_width)
    else:
        if (x_th_pixel_from_centre > 0): 
            # if the x-th pixel from the centre is positive, minus half the pixel size
            x_position = (x_th_pixel_from_centre * pixel_width) - (pixel_width / 2.0)
        else:
            # if the x-th pixel from the centre is negative, add half the pixel size
            x_position = (x_th_pixel_from_centre * pixel_width) + (pixel_width / 2.0)

    # Calculate y position
    image_height_odd = not ((num_y_pixels % 2.0) == 0.0)
    if image_height_odd:
        y_position = (y_th_pixel_from_centre * pixel_height)
    else:
        if (y_th_pixel_from_centre > 0): 
            # if the y-th pixel from the centre is positive, minus half the pixel size
            y_position = (y_th_pixel_from_centre * pixel_height) - (pixel_height / 2.0)
        else:
            # if the y-th pixel from the centre is negative, add half the pixel size
            y_position = (y_th_pixel_from_centre * pixel_height) + (pixel_height / 2.0)
        
    vector_length = math.sqrt((x_position ** 2.0) + (y_position ** 2.0) + (distance_to_near_plane ** 2.0))

    return ((x_position / vector_length), (y_position / vector_length), (distance_to_near_plane / vector_length))


def generate_unit_vector_map(pixel_size_x,
                             pixel_size_y,
                             num_x_pixels,
                             num_y_pixels,
                             distance_to_near_plane):
    """Generates a unit vector map,
       useful for pixel projection.

    i.e. Generates a unit vector for each pixel in an image,
         where the unit vector represents the ray along which
         the pixel is projected.

    Returns:
        2d array of 3-tuples: The unit vector map
                              e.g. for a 4x3 image it returns 
                              [[(,,), (,,), (,,), (,,)],
                               [(,,), (,,), (,,), (,,)],
                               [(,,), (,,), (,,), (,,)]]
    """

    image_width_odd = not ((num_x_pixels % 2.0) == 0.0)
    image_height_odd = not ((num_y_pixels % 2.0) == 0.0)
    half_width_floored = math.floor(num_x_pixels / 2)
    half_height_floored = math.floor(num_y_pixels / 2)

    unit_vector_map = []
    for x_th_pixel in xrange(num_x_pixels):
        unit_vector_map.append([])
        for y_th_pixel in xrange(num_y_pixels):
            # center x
            x_th_pixel_from_centre = x_th_pixel - half_width_floored
            if x_th_pixel_from_centre >= 0 and (not image_width_odd):
                # this skips 0 for even width images
                x_th_pixel_from_centre = (x_th_pixel - half_width_floored) + 1
                
            # center y
            y_th_pixel_from_centre = y_th_pixel - half_height_floored
            if y_th_pixel_from_centre >= 0 and (not image_height_odd):
                # this skips 0 for even height images
                y_th_pixel_from_centre = (y_th_pixel - half_height_floored) + 1
            # Invert y because top of image
            y_th_pixel_from_centre = -y_th_pixel_from_centre

            unit_vector_map[x_th_pixel].append(calc_unit_vector(pixel_size_x,
                                                                pixel_size_y,
                                                                x_th_pixel_from_centre,
                                                                y_th_pixel_from_centre,
                                                                num_x_pixels,
                                                                num_y_pixels,
                                                                distance_to_near_plane))
    return unit_vector_map


def projected_pixel(unit_vector_map,
                    x_th,
                    y_th,
                    depth):
    """Calculate the true (x, y, z) position of the projected pixel.
       NOTE: This method does a 2D translation, so that the center of the image
             is centered at (0, 0, 0)

    Args:
        unit_vector_map: Unit vector map generated by generate_unit_vector_map
        x_th (int): The x position of the pixel in the image
                    in the range 0 to (image_width - 1), where 0 is the left
        y_th (int): The y position of the pixel in the image
                    in the range 0 to (image_height - 1), where 0 is the top
        depth: depth value in meters

    Returns:
        3-tuple: The projected pixel, e.g. (x, y, z)
                 NOTE: x represents the horizontal (right is positive),
                       y represents the vertical (up is positive),
                       z represents going into the monitor as positive
    """
    
    unit_vector = unit_vector_map[x_th][y_th]  # get unit vector
    projected_pixel = (unit_vector[0] * depth, unit_vector[1] * depth, unit_vector[2] * depth)
    
    # return (x, y, z)
    return projected_pixel
# ________________________________________________________________


def setup_RF_map(image_height,
                 image_width,
                 map_height,
                 map_width):
    """Creates a RF (Receptive Field) map where RFs are equally spaced.
    An RF map is a map of (x, y) values, where each (x, y) 2-tuple is a single
    receptive field (RF)

    Example:
        3x3 example RF_map: [[(0,0),(0,1),(0,2)],
                             [(1,0),(1,1),(1,2)],
                             [(2,0),(2,1),(2,2)]]
        RF_map[0][2] = (0,2) which is the pixel position on the original image for the top right pixel of the retinal_encoded_image

    Args:
        image_height (int): The height of the original depth image
        image_width (int): The width of the original depth image
        map_height (int): Generally equal to the retinal encoded image height
        map_width (int): Generally equal to the retinal encoded image width
    """

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
    return RF_map


def sample_pixel_2d_norm_dist(RF,
                              standard_deviation_height,
                              standard_deviation_width,
                              times_to_try,
                              image_height,
                              image_width):
    """Generate (x, y) by sampling a 2D normal distribution
       https://docs.scipy.org/doc/numpy-1.14.0/reference/generated/numpy.random.normal.html

    Args:
        times_to_try (int): When the sample falls out of range (i.e. x=-3
                            isn't in the image), how many times should we resample
        image_height (int): The height of the original depth image
        image_width (int): The width of the original depth image
    """
    X = 0
    Y = 1

    # get x sample
    norm_dist_sample_x = np.random.normal(RF[X], standard_deviation_width)
    times_tried = 1
    while True:
        # if x is in bounds
        if (norm_dist_sample_x >= 0) and (norm_dist_sample_x < image_width):
            break
        else:
            # x is out of bounds, so generate new sample
            norm_dist_sample_x = np.random.normal(RF[X], standard_deviation_width)
            if times_tried > times_to_try:
                # if x is out of bounds to the left set it to 0
                if norm_dist_sample_x < 0:
                    norm_dist_sample_x = 0
                # else it is out of bounds to the right, so set it to (image_width - 1)
                else:
                    norm_dist_sample_x = image_width - 1

        times_tried = times_tried + 1

    # get y sample
    norm_dist_sample_y = np.random.normal(RF[Y], standard_deviation_height)
    times_tried = 1
    while True:
        # if y is in bounds
        if (norm_dist_sample_y >= 0) and (norm_dist_sample_y < image_height):
            break
        else:
            # y is out of bounds, so generate new sample
            norm_dist_sample_y = np.random.normal(RF[Y], standard_deviation_height)
            if times_tried > times_to_try:
                # if y is out of bounds to the top set it to 0
                if norm_dist_sample_y < 0:
                    norm_dist_sample_y = 0
                # else it is out of bounds to the bottom, so set it to (image_height - 1)
                else:
                    norm_dist_sample_y = image_height - 1

        times_tried = times_tried + 1

    return (int(norm_dist_sample_x), int(norm_dist_sample_y))


def setup_sampled_pixels_map(image_height,
                             image_width,
                             RF_map,
                             num_samples_per_RF):
    """Setup a map, where each point in the map is an array of sample pixels
    
    Example of a single point in the sampled_pixels_map:
        sampled_pixels_map[row][column] = [(x1,y1), (x2,y2), ...] (i.e. array of sampled pixels)
    """

    RF_map_rows = len(RF_map)
    RF_map_columns = len(RF_map[0])

    standard_deviation_height = int(image_height / (RF_map_rows * 3.0))
    standard_deviation_width = int(image_width / (RF_map_rows * 3.0))
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
                    sample_pixel_2d_norm_dist(RF_map[row][column],
                                              standard_deviation_height,
                                              standard_deviation_width,
                                              times_to_try,
                                              image_height,
                                              image_width)
                )

    return sampled_pixels_map


def draw_RF_map(image,
                RF_map,
                circle_radius,
                line_width,
                color):
    rows = len(RF_map)
    columns = len(RF_map[0])
    for row in xrange(rows):
        for column in xrange(columns):
            cv2.circle(image, RF_map[row][column],
                       circle_radius, color, line_width)
    
    return image


def draw_sampled_pixels_map(image,
                            sampled_pixels_map,
                            RF_map,
                            circle_radius,
                            line_width,
                            circle_color,
                            line_color):
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

    return image


def parabolic(f, x):
    """Quadratic interpolation for estimating the true position of an
    inter-sample maximum when nearby samples are known.

    From: https://gist.github.com/endolith/255291
   
    f is a vector and x is an index for that vector.
   
    Returns (vx, vy), the coordinates of the vertex of a parabola that goes
    through point x and its two neighbors.
   
    Example:
    Defining a vector f with a local maximum at index 3 (= 6), find local
    maximum if points 2, 3, and 4 actually defined a parabola.
   
    In [3]: f = [2, 3, 1, 6, 4, 2, 3, 1]
   
    In [4]: parabolic(f, np.argmax(f))
    Out[4]: (3.2142857142857144, 6.1607142857142856)
   
    """
    xv = 1/2. * (f[x-1] - f[x+1]) / (f[x-1] - 2 * f[x] + f[x+1]) + x
    yv = f[x] - 1/4. * (f[x-1] - f[x+1]) * (xv - x)
    return (xv, yv)


def freq_from_fft(sample_rate, signal):
    """Estimate frequency from peak of FFT

    From: https://gist.github.com/endolith/255291

    Usage:
        import scipy.io.wavfile as wav
        sample_rate, signal = wav.read("440.wav")
        print freq_from_fft(sample_rate, signal)

    Pro: Accurate, usually even more so than zero crossing counter
         (1000.000004 Hz for 1000 Hz, for instance). Due to parabolic
         interpolation being a very good fit for windowed log FFT peaks?
    Con: Doesn't find the right value if harmonics are stronger than
         fundamental, which is common. Better method would try to be smarter
         about identifying the fundamental, like template matching using the
         "two-way mismatch" (TWM) algorithm.
    """
    # Compute Fourier transform of windowed signal
    windowed = signal * blackmanharris(len(signal))
    f = np.fft.rfft(windowed)

    # Find the peak and interpolate to get a more accurate peak
    i = np.argmax(abs(f))  # Just use this for less-accurate, naive version
    true_i = parabolic(np.log(abs(f)), i)[0]

    # Convert to equivalent frequency
    return sample_rate * true_i / len(windowed)
