#!/usr/bin/env python

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


def depthCallback(depth_data):
    depth_image = bridge.imgmsg_to_cv2(depth_data, desired_encoding="32FC1")
    
    # log the depth image size once
    global counter
    if counter == 0:
        counter = 1
        rospy.loginfo(len(depth_image)) # 480 - height
        rospy.loginfo(len(depth_image[0])) # 640 - width

    # log out the distance to a specific point
    rospy.loginfo('Distance at 30x, 30y pixel: {}m'.format(depth_image[30][30]))

    # image -> output image
    # 640 x 480 -> 9 x 9
    depth_image_width = len(depth_image[0])
    depth_image_height = len(depth_image)
    output_image_width = 12
    output_image_height = 9

    # x axis, width
    x_inc = depth_image_width / output_image_width
    current_x = x_inc / 2

    # y axis, height
    y_inc = depth_image_height / output_image_height
    current_y = y_inc / 2

    output_image_array = []
    current_output_image_y = 0
    
    # y axis
    while (current_y < depth_image_height):
        # make actual pixel, i.e. no floats
        y_pixel = math.ceil(current_y) - 1

        # new row array
        output_image_array.append([])

        # x axis
        while (current_x < depth_image_width):
            # make actual pixel, i.e. no floats
            x_pixel = math.ceil(current_x) - 1

            # create the actual image array
            output_image_array[current_output_image_y].append(depth_image[y_pixel][x_pixel])
            
            # next x pixel
            current_x = current_x + x_inc
        
        # next y pixel
        current_y = current_y + y_inc
        # reset x pixels
        current_x = x_inc / 2
        current_output_image_y = current_output_image_y + 1

    # Create image from the image array
    # output_image_array = [[depth_image[30][30],depth_image[30][610]],[depth_image[450][30],depth_image[450][610]]]
    output_image = np.array(output_image_array, dtype=np.float32)
    image_pub.publish(bridge.cv2_to_imgmsg(output_image, "32FC1"))


def retinalEncoder():
    rospy.init_node('retinal_encoder', anonymous=True)

    # Get depth image from depth camera
    rospy.Subscriber("camera/depth/image", Image, depthCallback)

    rospy.spin()

if __name__ == '__main__':
    retinalEncoder()