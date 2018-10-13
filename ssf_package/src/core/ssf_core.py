
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

    depth_image[np.isnan(depth_image)] = 9999999

    # define criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    # apply kmeans()
    ret, label, k_center_values = cv2.kmeans(depth_image, num_clusters, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    res = k_center_values[label.flatten()]
    depth_image = res.reshape((rows, columns))

    depth_image[depth_image == 9999999] = np.nan

    return depth_image


def quantize_depth_image(image,
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

    num_quantization_levels = len(quantization_levels)

    image[image < quantization_levels[1]] = quantization_levels[0]
    
    for i in xrange(num_quantization_levels - 2):
        image[(image < quantization_levels[i + 2]) & (image >= quantization_levels[i + 1])] = quantization_levels[i + 1]
    
    image[image >= quantization_levels[num_quantization_levels - 1]] = quantization_levels[num_quantization_levels - 1]

    return image


def min_in_ndarray(ndarray):
    """Returns the min value in the given ndarray"""
    return np.nanmin(ndarray)


def mode_of_ndarray(ndarray):
    """Calculate the mode of a given ndarray.

    The mode is the most frequently occuring number found in a given set
    """
    flat_ndarray = ndarray.flatten()
    return stats.mode(flat_ndarray)[0][0]


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
        warnings.simplefilter("ignore", category=RuntimeWarning)
        mean_image = np.nanmean(combined_array, 0)

    return mean_image
