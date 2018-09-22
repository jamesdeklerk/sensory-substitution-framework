#!/usr/bin/env python

import rospy
import roslaunch
import rosparam

from dynamic_reconfigure.server import Server
from ssf_package.cfg import ParametersConfig


# TODO(for James): Maybe make part of ssf_core?
def launch_retinal_encoder(package, algorithm):
    executable = algorithm + "_retinal_encoder.py"
    node = roslaunch.core.Node(package, executable, name="retinal_encoder", output="screen")
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    process = launch.launch(node)
    
    return node


def parameter_changed_callback(config, level):

    print("retinal_encoder_algorithm: {}".format(config.retinal_encoder_algorithm))

    # TODO: check if file/node exists,
    #       if not, output error, retinal_encoder_... node not found,
    #       please make sure this was spelt correctly
    # TODO: Leave it to shutdown node automatically when node
    #       with same name launched, or manually shut down?
    launch_retinal_encoder("ssf_package", config.retinal_encoder_algorithm)

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
