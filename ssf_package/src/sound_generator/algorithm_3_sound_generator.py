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
alert_sound_sources = []
gain_scaled = 1.0
# ________________________________________________________________


def get_current_path():
    return os_path.dirname(os_path.abspath(__file__))


def pixel_position_scaling(pixel, x_scale, y_scale, z_scale, min_projected_z):
    pixel[0] = pixel[0] * (x_scale * 1.0)
    pixel[1] = pixel[1] * (y_scale * 1.0)
    # scales anything beyond the min projected z
    pixel[2] = min_projected_z + ((pixel[2] - min_projected_z) * (z_scale * 1.0))
    return (pixel[0], pixel[1], pixel[2])


def setup(retinal_encoded_image_cv2_format):
    global sound_folder_location, is_setup, sound_sources, unit_vector_map, gain_scaled, alert_sound_sources

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
    large_water_sample = load_wav_file(sound_folder_location + "large_water_sample.wav")
    water_lapping_wind_sample = load_wav_file(sound_folder_location + "water_lapping_wind_sample.wav")
    top = load_wav_file(sound_folder_location + "top.wav")
    top_middle = load_wav_file(sound_folder_location + "top_middle.wav")
    middle = load_wav_file(sound_folder_location + "middle.wav")
    bottom_middle = load_wav_file(sound_folder_location + "bottom_middle.wav")
    bottom = load_wav_file(sound_folder_location + "bottom.wav")
    beep_short = load_wav_file(sound_folder_location + "beep_short.wav")

    # To avoid clipping, the gain for each sound source needs to be
    # scaled down relative to the number of sound emitters
    gain_scaled = 1.0 / (retinal_encoded_image_width * retinal_encoded_image_height)
    gain_scaled = gain_scaled + 0.02

    # Setting up the sound sources for each receptive field (i.e. each
    # pixel in the retinal encoded image)
    for row in xrange(retinal_encoded_image_height):
        sound_sources.append([])
        for column in xrange(retinal_encoded_image_width):
            # A sound source is an object that emits sounds
            sound_sources[row].append(SoundSource(position=[0, 0, 0]))
            
            # Specifying if the source should loop the sound
            sound_sources[row][column].looping = True
            
            # Queueing appends the sound to the source for 
            # processing and playback
            if row == 0:
                sound_sources[row][column].queue(top)
            elif row == 1:
                sound_sources[row][column].queue(top_middle)
            elif row == 2:
                sound_sources[row][column].queue(middle)
            elif row == 3:
                sound_sources[row][column].queue(water_lapping_wind_sample)
            elif row == 4:
                sound_sources[row][column].queue(large_water_sample)

            # Scale gain
            sound_sources[row][column].gain = gain_scaled

            # Play the sound
            soundsink.play(sound_sources[row][column])

    # Setting up the sound sources for the minimum distance alert
    # 0 is left, 1 is right
    alert_sound_sources.append(SoundSource(position=[0, 0, 0]))
    alert_sound_sources[0].looping = True
    alert_sound_sources[0].queue(beep_short)
    alert_sound_sources[0].gain = 0.0
    soundsink.play(alert_sound_sources[0])
    alert_sound_sources.append(SoundSource(position=[0, 0, 0]))
    alert_sound_sources[1].looping = True
    alert_sound_sources[1].queue(beep_short)
    alert_sound_sources[1].gain = 0.0
    soundsink.play(alert_sound_sources[1])

    soundsink.update()

    is_setup = True


# ------------------------------------------------------------------------------------
def sound_generator_algorithm(retinal_encoded_image_cv2_format):
    retinal_encoded_image_width = len(retinal_encoded_image_cv2_format[0])
    retinal_encoded_image_height = len(retinal_encoded_image_cv2_format)

    # NOTE: PyAL uses the RHS coordinate system
    # Hence, the horizontal extent of the monitor represents the x-axis,
    # with right being positive. The vertical extent of the monitor represents
    # the y-axis, with up being positive; and from ones eyes going into the 
    # monitor represents the positive z-axis.

    if not is_setup:
        setup(retinal_encoded_image_cv2_format)

    # Calculate 
    max_pitch = 1.5
    min_pitch = 0.3

    beep_distance = 0.4
    left_beep = False
    right_beep = False

    for row in xrange(retinal_encoded_image_height):
        for column in xrange(retinal_encoded_image_width):
            depth = retinal_encoded_image_cv2_format[row][column]

            if np.isnan(depth) or                   \
               (depth == 0.0) or                    \
               (depth >= depth_camera_max_depth):
                sound_sources[row][column].gain = 0.0
            else:
                if (depth < beep_distance):
                    if (column <= 4):
                        left_beep = True
                    if (column >= 5):
                        right_beep = True

                # -----------------

                sound_sources[row][column].gain = gain_scaled

                # Calculate pitch based on distance
                current_depth_percentage = (depth - depth_camera_min_depth) / ((depth_camera_max_depth - depth_camera_min_depth) * 1.0)
                # Example:
                #   If the values are set to:
                #       max_pitch = 1.7
                #       min_pitch = 0.3
                #   Then:
                #       If current_depth_percentage = 0.0, pitch_based_on_distance = 1.7
                #       If current_depth_percentage = 0.5, pitch_based_on_distance = 1.0
                #       If current_depth_percentage = 1.0, pitch_based_on_distance = 0.3
                pitch_based_on_distance = max_pitch - ((max_pitch - min_pitch) * current_depth_percentage * 1.0)

                # If the sound isn't muted, update its pitch
                # dependent on the current depth 
                # NOTE: Setting the pitch stretches or compresses the sound
                #       by the given value. For example, if the pitch of a
                #       440Hz tone is set to 2.0, the tone played would be
                #       2 * 440Hz = 880Hz
                sound_sources[row][column].pitch = pitch_based_on_distance

                projected_min_depth = ssf_core.projected_pixel(unit_vector_map,
                                                            column,
                                                            row,
                                                            depth_camera_min_depth)[2]
        
                x_scale = 2.5
                y_scale = 1.0
                z_scale = 1.3
                # scales anything beyond the projected_min_depth,
                # scaling it along the ray
                z_power_scale = 2.0
                # NOTE: only the depth is scaled, and then x, y and z are projected
                #       according to that depth.
                depth = (projected_min_depth * z_scale) + (((depth - projected_min_depth) * z_scale)**(z_power_scale * 1.0))
                projected_pixel = ssf_core.projected_pixel(unit_vector_map,
                                                           column,
                                                           row,
                                                           depth)
        
                # Update the sound sources position based on the projected pixel
                sound_sources[row][column].position = [projected_pixel[0] * x_scale,
                                                       projected_pixel[1],
                                                       -projected_pixel[2]]

    if left_beep:
        alert_sound_sources[0].gain = 0.5
        alert_sound_sources[0].position = [-1, 0, -1]
    else:
        alert_sound_sources[0].gain = 0.0
    if right_beep:
        alert_sound_sources[1].gain = 0.5
        alert_sound_sources[1].position = [1, 0, -1]
    else:
        alert_sound_sources[1].gain = 0.0

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
