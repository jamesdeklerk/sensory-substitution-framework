#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import dynamic_reconfigure.client
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Audio imports
from openal.audio import SoundListener, SoundSink, SoundSource
from openal.loaders import load_wav_file

# importing the ssf_core module
import rospkg
from sys import path as system_path
from os import path as os_path
rospack = rospkg.RosPack()
core_package_path = os_path.join(rospack.get_path('ssf_package'), 'src', 'core')
system_path.append(core_package_path)
import ssf_core


# ----------------------------------------------------------------
#                            GLOBALS
# ----------------------------------------------------------------
# Algorithm name (from the filename)
file_name = os_path.splitext(os_path.basename(__file__))[0]
algorithm_name = file_name.replace("_sound_generator", "")
ssf_core.check_algorithm_name(algorithm_name)

# Input params
sg_input_params = rospy.get_param("/sg/input")
retinal_encoded_image_topic = sg_input_params["retinal_encoded_image"]["topic"]

# Depth camera params
depth_camera_params = rospy.get_param("/depth_camera")
depth_camera_min_depth = depth_camera_params["min_depth"]
depth_camera_max_depth = depth_camera_params["max_depth"]
depth_camera_h_fov = depth_camera_params["h_fov"]
depth_camera_v_fov = depth_camera_params["v_fov"]
depth_camera_crop_width_percentage = depth_camera_params["crop_width_percentage"]
depth_camera_crop_height_percentage = depth_camera_params["crop_height_percentage"]
cropped_h_fov, cropped_v_fov = ssf_core.calc_cropped_fov(depth_camera_h_fov,
                                                         depth_camera_v_fov,
                                                         depth_camera_crop_width_percentage,
                                                         depth_camera_crop_height_percentage)

# Algorithm specific globals
# TODO: Uncomment the next line if you use custom params for your algorithm
# sg_algorithm_params = rospy.get_param(algorithm_name + "/sg")
# TODO: Place your algorithms custom params below
# ...

# Other globals
bridge = CvBridge()
retinal_encoded_image_h_fov_rad = cropped_h_fov * (np.pi / 180.0)  # h_fov in rad
retinal_encoded_image_v_fov_rad = cropped_v_fov * (np.pi / 180.0)  # v_fov in rad
unit_vector_map = None
# Audio globals
soundsink = SoundSink()  # Opening output device
sound_folder_location = None
is_setup = False
sound_sources = []
# ________________________________________________________________


def get_current_path():
    return os_path.dirname(os_path.abspath(__file__))


def setup(retinal_encoded_image_cv2_format):
    global sound_folder_location, is_setup, sound_sources, unit_vector_map

    retinal_encoded_image_width = len(retinal_encoded_image_cv2_format[0])
    retinal_encoded_image_height = len(retinal_encoded_image_cv2_format)

    # Generate unit vector map
    distance_to_near_plane = 0.5 # arbitrary distance
    pixel_width, pixel_height = ssf_core.calc_pixel_size(distance_to_near_plane,
                                                         retinal_encoded_image_h_fov_rad,
                                                         retinal_encoded_image_v_fov_rad,
                                                         retinal_encoded_image_width,
                                                         retinal_encoded_image_height)
    unit_vector_map = ssf_core.generate_unit_vector_map(pixel_width,
                                                        pixel_height,
                                                        retinal_encoded_image_width,
                                                        retinal_encoded_image_height,
                                                        distance_to_near_plane)

    sound_folder_location = get_current_path() + "/sound_files/"
    soundsink.activate()

    # Setting up the listner using the defaults specified
    # here: https://media.readthedocs.org/pdf/pyal/latest/pyal.pdf
    listener = SoundListener()
    listener.position = (0, 0, 0)
    listener.velocity = (0, 0, 0)
    # (x-direction, y-direction, z-direction, x-rotation, y-rotation, z-rotation)
    listener.orientation = (0, 0, -1, 0, 1, 0)

    # Load the audio
    sample_audio = load_wav_file(sound_folder_location + "sample_audio.wav")

    # Setting up the sound sources for each receptive field (i.e. each
    # pixel in the retinal encoded image)
    for row in xrange(retinal_encoded_image_height):
        sound_sources.append([])
        for column in xrange(retinal_encoded_image_width):
            # A sound source is an object that emits sounds
            sound_sources[row].append(SoundSource(position=[0, 0, -1]))
            
            # Specifying if the source should loop the sound
            sound_sources[row][column].looping = True
            
            # Queueing appends the sound to the source for 
            # processing and playback
            sound_sources[row][column].queue(sample_audio)

            # Setting the gain for each source:
            #   Assuming the total gain should sum up to the max of a single
            #   sample_audio file, then each sound sources gain should be
            #   divided by the number of sound emitters.
            sound_sources[row][column].gain = sound_sources[row][column].gain       \
                                              / (retinal_encoded_image_height       \
                                              * retinal_encoded_image_width)

            soundsink.play(sound_sources[row][column])

    soundsink.update()

    is_setup = True


# ------------------------------------------------------------------------------------
# TODO: Replace with your algorithm
#       The default template algorithm simply sets up the sound_sources
#       to be the same sample sound, and then moves each sound source in
#       3D space using ssf_core.projected_pixel
def sound_generator_algorithm(retinal_encoded_image_cv2_format):
    retinal_encoded_image_width = len(retinal_encoded_image_cv2_format[0])
    retinal_encoded_image_height = len(retinal_encoded_image_cv2_format)

    if not is_setup:
        setup(retinal_encoded_image_cv2_format)
    
    for row in xrange(retinal_encoded_image_height):
        for column in xrange(retinal_encoded_image_width):

            depth = retinal_encoded_image_cv2_format[row][column]

            # This will correctly project the pixel in 3D space (along the
            # approproate ray), considering the horizontal and vertical FoV
            # as well as the x, y and z (depth) position of the pixel
            # 
            # NOTE: x represents the horizontal (right is positive),
            #       y represents the vertical (up is positive),
            #       z represents going into the monitor (forward) as positive
            projected_pixel = ssf_core.projected_pixel(unit_vector_map,
                                                       column,
                                                       row,
                                                       depth)
            
            # NOTE: PyAL uses the RHS coordinate system, i.e.
            #           x represents the horizontal (right is positive),
            #           y represents the vertical (up is positive),
            #           z represents coming out of the monitor as positive
            # 
            # Hence: When positioning a sound source, using the projected_pixel,
            #        the z-axis must be flipped (i.e. made negative)
            sound_sources[row][column].position = [projected_pixel[0],
                                                   projected_pixel[1],
                                                   -projected_pixel[2]]

    soundsink.update()
# ____________________________________________________________________________________


def retinal_encoded_image_callback(retinal_encoded_image_imgmsg_format):
    # convert from ROS Image message to OpenCV Image
    # NOTE: An OpenCV Image is essentially a numpy array (super cool)
    retinal_encoded_image_cv2_format = bridge.imgmsg_to_cv2(retinal_encoded_image_imgmsg_format, desired_encoding="32FC1")

    # --------------------------------------------------------------------------------
    # TODO: Replace with your algorithm
    sound_generator_algorithm(retinal_encoded_image_cv2_format)
    # ________________________________________________________________________________


def parameter_changed_callback(config):
    pass


def main():
    rospy.init_node(file_name)
    print("NODE RUNNING: " + file_name)

    # Connect to dynamic_reconfigure server
    dynamic_reconfigure.client.Client("dynamic_parameters_server",
                                      timeout=300, config_callback=parameter_changed_callback)

    # Get retinal encoded image from retinal encoder
    rospy.Subscriber(retinal_encoded_image_topic, Image, retinal_encoded_image_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
