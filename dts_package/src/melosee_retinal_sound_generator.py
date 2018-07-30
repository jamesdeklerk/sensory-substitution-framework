#!/usr/bin/env python

# pip install numpy

import math
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Audio imports
from openal.audio import SoundSink, SoundSource, SoundListener
from openal.loaders import load_wav_file

import random
import numpy as np

bridge = CvBridge()

# Audio globals
soundSources = []
soundSourcesSetup = False
soundsink = SoundSink() # Opening output device
retinal_encoded_image_width = 0
retinal_encoded_image_height = 0
# C-major, the scale of just intonation would be: C=1/1 D=9/8 E=5/4 F= 4/3 G=3/2 A=5/3 B=15/8 C=2/1
C_4 = 264.0
D = 297.0 # C_4 * (9.0/8.0)
E = 330.0 # C_4 * (5.0/4.0)
F = 352.0 # C_4 * (4.0/3.0)
G = 396.0 # C_4 * (3.0/2.0)
A = 440.0 # C_4 * (5.0/3.0)
B = 495.0 # C_4 * (15.0/8.0)
C_5 = 528.0 # C_4 * (2.0/1.0)

def generate_sound_file_name(frequency):
    return str(int(frequency)) + '.wav'

def callback(retinal_encoded_data):
    retinal_encoded_image = bridge.imgmsg_to_cv2(retinal_encoded_data, desired_encoding="32FC1")

    # Loading the audio data
    folder_location = "sound_files/"
    row_one_audio = load_wav_file(folder_location + generate_sound_file_name(C_5)) # top
    row_two_audio = load_wav_file(folder_location + generate_sound_file_name(B))
    row_three_audio = load_wav_file(folder_location + generate_sound_file_name(A))
    row_four_audio = load_wav_file(folder_location + generate_sound_file_name(G))
    row_five_audio = load_wav_file(folder_location + generate_sound_file_name(F))
    row_six_audio = load_wav_file(folder_location + generate_sound_file_name(E))
    row_seven_audio = load_wav_file(folder_location + generate_sound_file_name(D))
    row_eight_audio = load_wav_file(folder_location + generate_sound_file_name(C_4)) # bottom

    global retinal_encoded_image_width
    global retinal_encoded_image_height
    global soundSources
    global soundSourcesSetup
    if not soundSourcesSetup:
        retinal_encoded_image_width = len(retinal_encoded_image[0])
        retinal_encoded_image_height = len(retinal_encoded_image)

        soundsink.activate()

        # Setting up the listner (can actually comment this all out)
        listener = SoundListener()
        listener.position = (0, 0, 0)               # default = (0, 0, 0)
        listener.velocity = (0, 0, 0)               # default = (0, 0, 0)
        # (x-direction, y-direction, z-direction, x-rotation, y-rotation, z-rotation)
        listener.orientation = (0, 0, -1, 0, 1, 0)  # default = (0, 0, -1, 0, 1, 0)

        # Setup sound sources for each "receptive field"
        # Create array of sound sources
        x_pos = 0
        y_pos = 0
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
                elif y ==  2:
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
                soundSources[y][x].position = [x - (retinal_encoded_image_width / 2), y - (retinal_encoded_image_height / 2), -random.randint(1, 9)]

        soundsink.update()
        print('soundSources have been setup')

        soundSourcesSetup = True
    
    # TODO: update positions of sound sources
    for y in xrange(retinal_encoded_image_height):
        for x in xrange(retinal_encoded_image_width):
            x_scale_factor = 3
            x_pos = ((x + 0.5) - (retinal_encoded_image_width / 2)) * x_scale_factor          # left is negative
            y_scale_factor = 3
            y_pos = (-((y + 0.5) - (retinal_encoded_image_height / 2))) * y_scale_factor    # up is positive
            z_scale_factor = 3
            z_pos = retinal_encoded_image[y][x] * z_scale_factor
            if math.isnan(z_pos):
                z_pos = 1000        # basically too far to hear
            soundSources[y][x].position = [x_pos, y_pos, z_pos]
    
    soundsink.update()

def soundGenerator():
    rospy.init_node('sound_generator', anonymous=True)
    print('NODE RUNNING: sound_generator')

    # subscribe to retinal encoded image
    rospy.Subscriber("retinal_encoded_image", Image, callback)

    rospy.spin()

if __name__ == '__main__':
    soundGenerator()