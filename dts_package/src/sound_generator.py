#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


bridge = CvBridge()
soundSourcesSetup = False


def callback(retinal_encoded_data):
    retinal_encoded_image = bridge.imgmsg_to_cv2(retinal_encoded_data, desired_encoding="32FC1")

    global soundSourcesSetup
    if not soundSourcesSetup:
        width = len(retinal_encoded_image[0])
        height = len(retinal_encoded_image)

        # TODO: setup sound sources for each "receptive field"

        soundSourcesSetup = True
    
    # TODO: update positions of sound sources


def soundGenerator():
    rospy.init_node('sound_generator', anonymous=True)

    # subscribe to retinal encoded image
    rospy.Subscriber("retinal_encoded_image", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    soundGenerator()