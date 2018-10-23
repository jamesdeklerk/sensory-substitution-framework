#!/usr/bin/env python

# pip install numpy

import math
import os
import random
import sys

import cv2
import numpy as np
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String

# Audio imports
from openal.audio import SoundListener, SoundSink, SoundSource
from openal.loaders import load_wav_file

sg_input_params = rospy.get_param("/sg/input")
retinal_encoded_image_topic = sg_input_params["retinal_encoded_image"]["topic"]

# Depth camera params
depth_camera_params = rospy.get_param("/depth_camera")
depth_camera_min_depth = depth_camera_params["min_depth"]
depth_camera_max_depth = depth_camera_params["max_depth"]

bridge = CvBridge()

# Audio globals
soundSources = []
soundSourcesSetup = False
soundsink = SoundSink()  # Opening output device

# C-major, the scale of just intonation would be: C=1/1 D=9/8 E=5/4 F= 4/3 G=3/2 A=5/3 B=15/8 C=2/1
C_4 = 264.0
D = 297.0  # C_4 * (9.0/8.0)
E = 330.0  # C_4 * (5.0/4.0)
F = 352.0  # C_4 * (4.0/3.0)
G = 396.0  # C_4 * (3.0/2.0)
A = 440.0  # C_4 * (5.0/3.0)
B = 495.0  # C_4 * (15.0/8.0)
C_5 = 528.0  # C_4 * (2.0/1.0)
min_z = 1000000000
max_z = -1000000000


# TODO: Check if width or height of image chaged, if it did, update it
#       print("reconfigure sound_generator for new depth image")


def get_current_path():
    return os.path.dirname(os.path.abspath(__file__))


sound_folder_location = get_current_path() + "/sound_files/"


def generate_sound_file_name(frequency):
    return str(int(frequency)) + '.wav'


def callback(retinal_encoded_data):
    retinal_encoded_image = bridge.imgmsg_to_cv2(
        retinal_encoded_data, desired_encoding="32FC1")

    retinal_encoded_image_width = len(retinal_encoded_image[0])
    retinal_encoded_image_height = len(retinal_encoded_image)

    if (retinal_encoded_image_width != 8 or retinal_encoded_image_height != 8):
        rospy.logerr("The retinal_encoded_image must be an 8 x 8 image!!!")

    # Loading the audio data
    row_one_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(C_5))  # top
    row_two_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(B))
    row_three_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(A))
    row_four_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(G))
    row_five_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(F))
    row_six_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(E))
    row_seven_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(D))
    row_eight_audio = load_wav_file(
        sound_folder_location + generate_sound_file_name(C_4))  # bottom

    global soundSources
    global soundSourcesSetup
    if not soundSourcesSetup:
        soundsink.activate()

        # Setting up the listner (can actually comment this all out)
        listener = SoundListener()
        listener.position = (0, 0, 0)               # default = (0, 0, 0)
        listener.velocity = (0, 0, 0)               # default = (0, 0, 0)
        # (x-direction, y-direction, z-direction, x-rotation, y-rotation, z-rotation)
        # default = (0, 0, -1, 0, 1, 0)
        listener.orientation = (0, 0, -1, 0, 1, 0)

        # Setup sound sources for each "receptive field"
        # Create array of sound sources
        for y in xrange(retinal_encoded_image_height):
            soundSources.append([])
            for x in xrange(retinal_encoded_image_width):
                # A SoundSource is an object that emits sounds
                soundSources[y].append(SoundSource(position=[0, 0, 0]))
                # Specifying if the source should loop the sound
                soundSources[y][x].looping = True
                # Queueing appends the sound to the source for processing and playback
                if y == 0:
                    soundSources[y][x].queue(row_one_audio)
                elif y == 1:
                    soundSources[y][x].queue(row_two_audio)
                elif y == 2:
                    soundSources[y][x].queue(row_three_audio)
                elif y == 3:
                    soundSources[y][x].queue(row_four_audio)
                elif y == 4:
                    soundSources[y][x].queue(row_five_audio)
                elif y == 5:
                    soundSources[y][x].queue(row_six_audio)
                elif y == 6:
                    soundSources[y][x].queue(row_seven_audio)
                elif y == 7:
                    soundSources[y][x].queue(row_eight_audio)
                # Informing the SoundSink about the SoundSource so it knows a new sound emitter is available
                soundsink.play(soundSources[y][x])

                # TODO: fix start position
                soundSources[y][x].position = [x - (retinal_encoded_image_width / 2), y -
                                               (retinal_encoded_image_height / 2), -random.randint(1, 9)]

        soundsink.update()
        print('soundSources have been setup')

        soundSourcesSetup = True

    # TODO: update positions of sound sources
    x_scale_factor = 0.2
    y_scale_factor = 0.0  # set to zero, since MeloSee doesn't use depth
    z_power_scale_factor = 3.0

    gain_scaled = 1.0 / (retinal_encoded_image_width * retinal_encoded_image_height)

    x_pos = 0
    y_pos = 0
    for row in xrange(retinal_encoded_image_height):
        for column in xrange(retinal_encoded_image_width):
            # center x
            x_pos = column - ((retinal_encoded_image_width - 1.0) / 2.0)
            # scale x
            x_pos = x_pos * x_scale_factor  # right is positive

            # center y
            y_pos = row - ((retinal_encoded_image_height - 1.0) / 2.0)
            # flip to correct orientation
            y_pos = -y_pos
            # scale y
            y_pos = y_pos * y_scale_factor  # up is positive
            
            # distance
            z_pos = retinal_encoded_image[row][column]

            # Gain settings, dependant on z
            if math.isnan(z_pos) or                     \
               (z_pos == 0.0) or                        \
               (z_pos >= depth_camera_max_depth):
                soundSources[row][column].gain = 0.0
            else:
                soundSources[row][column].gain = gain_scaled

            # NB: z scaling is done after gain settings
            z_pos = depth_camera_min_depth + ((z_pos - depth_camera_min_depth)**(z_power_scale_factor * 1.0))

            soundSources[row][column].position = [x_pos, y_pos, -z_pos]

    soundsink.update()


def soundGenerator():
    rospy.init_node('melosee_sound_generator')
    print('NODE RUNNING: melosee_sound_generator')

    # subscribe to retinal encoded image
    rospy.Subscriber(retinal_encoded_image_topic, Image, callback)

    rospy.spin()


if __name__ == '__main__':
    soundGenerator()
