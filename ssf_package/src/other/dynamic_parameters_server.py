#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from ssf_package.cfg import ParametersConfig


def parameter_changed_callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
            {str_param}, {bool_param}, {size}""".format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("dynamic_parameters_server", anonymous=False)

    server = Server(ParametersConfig, parameter_changed_callback)
    rospy.spin()
