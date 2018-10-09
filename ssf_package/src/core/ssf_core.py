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
    """

    # e.g. 20 to 100 with 8 levels
    # diff = 100 - 20 = 80
    # 80 / (8 - 1) = 11.428...
    # 20.0, 31.4, 42.8, 54.2, 65.7, 77.1, 88.5, 100.0

    # 14    > 20
    # 25    > 20
    # 51    > 42.8
    # 88.5  > 88.5
    # 200   > 100

    quantization_levels = np.linspace(min_depth, max_depth, num_quantization_levels)
    
    def quantize(value):
        
        count = 0
        while value >= quantization_levels[count]:
            count = count + 1
            if count >= num_quantization_levels:
                break

        count = count - 1
        count = 0 if count < 0 else count
        return quantization_levels[count]

    return np.vectorize(quantize)(image)

