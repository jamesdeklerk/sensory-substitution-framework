# This module contains commonly used functions for the
# Sensory Substitution Framework (SSF)

import math
import numpy as np


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


def quantize_depth_image(image, min_depth, max_depth, num_quantization_levels):
    """Quantize a given depth image.

    Example:
    - For these params:
        min_depth = 20
        max_depth = 100
        num_quantization_levels = 8
    - This will be generated:
        quantization_levels = [20.0, 31.4, 42.8, 54.2, 65.7, 77.1, 88.5, 100.0]
    - For each depth value in the image:
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

