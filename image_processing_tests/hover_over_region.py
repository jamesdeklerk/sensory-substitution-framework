# pip install opencv-python
# pip install numpy
# pip install matplotlib

import numpy as np
import cv2
import time

GRAYSCALE_MAX = 1 # for cv2 it's 1 not 255

WIDTH = 10
HEIGHT = 10
MIN_DISTANCE = 0.5  # in meters
MAX_DISTANCE = 2.5  # in meters
DEPTH_SCALE_FACTOR = (GRAYSCALE_MAX / MAX_DISTANCE)

WINDOW_HEIGHT = 300
WINDOW_WIDTH = 300


def generate_depth_image(height, width, default_value):
    image_array = []

    for row in xrange(height):
        # add row
        image_array.append([])
        for column in xrange(width):
            image_array[row].append(default_value)

    return np.array(image_array, dtype=np.float32)


def convert_depth_image_to_image(depth_image, depth_scale_factor):
    width = len(depth_image)
    height = len(depth_image)

    for row in xrange(height):
        for column in xrange(width):
            depth_image[row][column] = depth_image[row][column] * depth_scale_factor

    return depth_image


depth_image = generate_depth_image(HEIGHT, WIDTH, MAX_DISTANCE / 2)
image = convert_depth_image_to_image(depth_image, DEPTH_SCALE_FACTOR)
drawn_on_image = image.copy()
ix, iy = 0, 0
dragging = False
dragging_start = 0
current_distance = 0.5

def draw_circle(event, x, y, flags, param):
    global image, drawn_on_image, current_distance, dragging, dragging_start, ix, iy
    
    drawn_on_image = image.copy()
    drawn_on_image[y][x] = (current_distance / MAX_DISTANCE) * GRAYSCALE_MAX

    ix, iy = x, y

    # TODO: click and drag to increase or decrease depth
    if event == cv2.EVENT_LBUTTONDOWN:
        print("dragging started")
        dragging = True
        dragging_start = y

    if event == cv2.EVENT_LBUTTONUP:
        print("dragging stopped")
        dragging = False

        drag_distance = ((dragging_start - y) * 1.0) / (HEIGHT - 1) * MAX_DISTANCE
        current_distance = current_distance + drag_distance

        if current_distance > MAX_DISTANCE:
            current_distance = MAX_DISTANCE
        if current_distance < MIN_DISTANCE:
            current_distance = MIN_DISTANCE

        print("distance: {}".format(current_distance))

cv2.namedWindow("image", cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', WINDOW_WIDTH, WINDOW_HEIGHT)
cv2.setMouseCallback('image', draw_circle)

while True:
    cv2.imshow('image', drawn_on_image)
    key = cv2.waitKey(20) & 0xFF
    if key == 27:
        break
    elif key == ord('a'):
        print ix,iy

cv2.destroyAllWindows()
