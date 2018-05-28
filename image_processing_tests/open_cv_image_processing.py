# pip install opencv-python
# pip install numpy
# pip install matplotlib

import numpy as np
import cv2

# https://stackoverflow.com/questions/4195453/how-to-resize-an-image-with-opencv2-0-and-python2-6
# https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#void%20remap(InputArray%20src,%20OutputArray%20dst,%20InputArray%20map1,%20InputArray%20map2,%20int%20interpolation,%20int%20borderMode,%20const%20Scalar&%20borderValue)
# Python: cv2.resize(src, dsize[, dst[, fx[, fy[, interpolation]]]])

# explicitly specify dsize=dst.size(); fx and fy will be computed from that.
#       resize(src, dst, dst.size(), 0, 0, interpolation);
# specify fx and fy and let the function compute the destination image size.
#       resize(src, dst, Size(), 0.5, 0.5, interpolation);

scaled_width = 9
scaled_height = 9

window_size = 300
window_size_extra_width = 100
window_size_extra_height = 80

# Load an color image in grayscale
original = cv2.imread('depth_images/dont_use_in_masters.jpg',0)
small_default = cv2.resize(original, (scaled_width, scaled_height)) 
INTER_NEAREST = cv2.resize(original, (scaled_width, scaled_height), interpolation=cv2.INTER_NEAREST)
INTER_LINEAR = cv2.resize(original, (scaled_width, scaled_height), interpolation=cv2.INTER_LINEAR)
INTER_AREA = cv2.resize(original, (scaled_width, scaled_height), interpolation=cv2.INTER_AREA)
INTER_CUBIC = cv2.resize(original, (scaled_width, scaled_height), interpolation=cv2.INTER_CUBIC)
INTER_LANCZOS4 = cv2.resize(original, (scaled_width, scaled_height), interpolation=cv2.INTER_LANCZOS4)

cv2.namedWindow('original', cv2.WINDOW_NORMAL)
cv2.resizeWindow('original', window_size, window_size)
cv2.moveWindow('original', int(window_size * 0), int((window_size * 0.5) + (window_size_extra_height * 0.5)))
cv2.imshow('original',original)

cv2.namedWindow('small_default', cv2.WINDOW_NORMAL)
cv2.resizeWindow('small_default', window_size, window_size)
cv2.moveWindow('small_default', (window_size * 1) + window_size_extra_width, 0)
cv2.imshow('small_default',small_default)

cv2.namedWindow('INTER_NEAREST', cv2.WINDOW_NORMAL)
cv2.resizeWindow('INTER_NEAREST', window_size, window_size)
cv2.moveWindow('INTER_NEAREST', (window_size * 2) + window_size_extra_width, 0)
cv2.imshow('INTER_NEAREST',INTER_NEAREST)

cv2.namedWindow('INTER_LINEAR', cv2.WINDOW_NORMAL)
cv2.resizeWindow('INTER_LINEAR', window_size, window_size)
cv2.moveWindow('INTER_LINEAR', (window_size * 3) + window_size_extra_width, 0)
cv2.imshow('INTER_LINEAR',INTER_LINEAR)

cv2.namedWindow('INTER_AREA', cv2.WINDOW_NORMAL)
cv2.resizeWindow('INTER_AREA', window_size, window_size)
cv2.moveWindow('INTER_AREA', (window_size * 1) + window_size_extra_width, (window_size * 1) + window_size_extra_height)
cv2.imshow('INTER_AREA',INTER_AREA)

cv2.namedWindow('INTER_CUBIC', cv2.WINDOW_NORMAL)
cv2.resizeWindow('INTER_CUBIC', window_size, window_size)
cv2.moveWindow('INTER_CUBIC', (window_size * 2) + window_size_extra_width, (window_size * 1) + window_size_extra_height)
cv2.imshow('INTER_CUBIC',INTER_CUBIC)

cv2.namedWindow('INTER_LANCZOS4', cv2.WINDOW_NORMAL)
cv2.resizeWindow('INTER_LANCZOS4', window_size, window_size)
cv2.moveWindow('INTER_LANCZOS4', (window_size * 3) + window_size_extra_width, (window_size * 1) + window_size_extra_height)
cv2.imshow('INTER_LANCZOS4',INTER_LANCZOS4)

# click Esc key (with one of the image windows in focus) to stop
cv2.waitKey(0)
cv2.destroyAllWindows()
exit()
