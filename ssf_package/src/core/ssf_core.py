# This module contains commonly used functions for the
# Sensory Substitution Framework (SSF)

import rospy
import math
import warnings
import numpy as np
from scipy import stats


def crop_image(image, crop_width_per=0, crop_height_per=0):
    """Crop the given image.

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

    return image[y_start_index:(y_end_index + 1), x_start_index:(x_end_index + 1)]


def quantize_depth_image(image,
                         min_depth,
                         max_depth,
                         num_quantization_levels):
    """Quantize a given depth image.

    Example:
        For these params:
            min_depth = 20
            max_depth = 100
            num_quantization_levels = 8
        This will be generated:
            quantization_levels = [20.0, 31.4, 42.8, 54.2, 65.7, 77.1, 88.5, 100.0]
        For each depth value in the image:
            0    to <31.4   mapped to   20.0
            31.4 to <42.8   mapped to   31.4
            42.8 to <54.2   mapped to   42.8
            54.2 to <65.7   mapped to   54.2
            65.7 to <77.1   mapped to   65.7
            77.1 to <88.5   mapped to   77.1
            88.5 to <100.0  mapped to   88.8
            >=100.0         mapped to   100.0

    """

    quantization_levels = np.linspace(min_depth, max_depth, num_quantization_levels)

    image[image < quantization_levels[1]] = quantization_levels[0]
    
    for i in xrange(num_quantization_levels - 2):
        image[(image < quantization_levels[i + 2]) & (image >= quantization_levels[i + 1])] = quantization_levels[i + 1]
    
    image[image >= quantization_levels[num_quantization_levels - 1]] = quantization_levels[num_quantization_levels - 1]

    return image


def mode(nparray):
    flat_nparray = nparray.flatten()
    return stats.mode(flat_nparray)[0][0]


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

    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        mean_image = np.nanmean(combined_array, 0)

    return mean_image


def min_value_in_section(nparray,
                         x_start_index, 
                         x_end_index, 
                         y_start_index, 
                         y_end_index):
    """Returns the min value in the specified section of the given nparray

    NOTE:
    - x_end_index is inclusive of the index given
    - y_end_index is inclusive of the index given
    """
    
    nparray_subsection = nparray[y_start_index:(y_end_index + 1), x_start_index:(x_end_index + 1)]

    return np.nanmin(nparray_subsection)
