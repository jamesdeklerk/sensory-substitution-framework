#!/usr/bin/env python

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

bridge = CvBridge()

# Audio globals
soundSources = []
soundSourcesSetup = False
soundsink = SoundSink() # Opening output device
retinal_encoded_image_width = 0
retinal_encoded_image_height = 0

def callback(retinal_encoded_data):
    retinal_encoded_image = bridge.imgmsg_to_cv2(retinal_encoded_data, desired_encoding="32FC1")

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

        # Loading the audio data
        audioData = load_wav_file("audio/hey.wav")

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
                soundSources[y][x].queue(audioData)
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