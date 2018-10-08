#!/usr/bin/env python

import rospy
import roslaunch

from dynamic_reconfigure.server import Server
from ssf_package.cfg import ParametersConfig


current_retinal_encoder_algorithm = ""
current_sound_generator_algorithm = ""


# TODO(for James): Maybe make part of ssf_core?
def launch_node(package_name, executable_file, node_name, output_to_screen):
    
    if (output_to_screen):
        node = roslaunch.core.Node(package_name, executable_file, name=node_name, output="screen")
    else:
        node = roslaunch.core.Node(package_name, executable_file, name=node_name)

    # This will automatically shutdown a node with same name launched
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)

    return node


def parameter_changed_callback(config, level):

    global current_retinal_encoder_algorithm, current_sound_generator_algorithm

    # Update the retinal encoder algorithm if need be
    if (current_retinal_encoder_algorithm != config.re_algorithm):
        try:
            launch_node("ssf_package", config.re_algorithm + "_retinal_encoder.py",
                        "retinal_encoder", True)
        except Exception:
            rospy.logerr("Looks like something went wrong running that re_algorithm, are you sure '" +
                         config.re_algorithm + "' is a valid algorithm?")
        else:
            current_retinal_encoder_algorithm = config.re_algorithm
            rospy.logwarn("retinal_encoder_algorithm changed to: " + current_retinal_encoder_algorithm)

    # Update the sound generator algorithm if need be
    if (current_sound_generator_algorithm != config.sg_algorithm):
        try:
            launch_node("ssf_package", config.sg_algorithm + "_sound_generator.py",
                        "sound_generator", True)
        except Exception:
            rospy.logerr("Looks like something went wrong running that sg_algorithm, are you sure '" +
                         config.sg_algorithm + "' is a valid algorithm?")
        else:
            current_sound_generator_algorithm = config.sg_algorithm
            rospy.logwarn("sound_generator_algorithm changed to: " + current_sound_generator_algorithm)

    return config


def main():
    rospy.init_node("dynamic_parameters_server", anonymous=False)

    server = Server(ParametersConfig, parameter_changed_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
