#!/usr/bin/env python

# For setup:
# pip install -U PySide
# pip install pyqtgraph
# pip install pyopengl

import rospy
from sensor_msgs.msg import Image
import dynamic_reconfigure.client
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import time
import sys
import math

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

# Input params
sg_input_params = rospy.get_param("/sg/input")
retinal_encoded_image_topic = sg_input_params["retinal_encoded_image"]["topic"]

# Depth camera params
depth_camera_params = rospy.get_param("/depth_camera")
depth_camera_h_fov = depth_camera_params["h_fov"]
depth_camera_v_fov = depth_camera_params["v_fov"]
depth_camera_crop_width_percentage = depth_camera_params["crop_width_percentage"]
depth_camera_crop_height_percentage = depth_camera_params["crop_height_percentage"]
cropped_h_fov, cropped_v_fov = ssf_core.calc_cropped_fov(depth_camera_h_fov,
                                                         depth_camera_v_fov,
                                                         depth_camera_crop_width_percentage,
                                                         depth_camera_crop_height_percentage)

# Other globals
bridge = CvBridge()
setup = False
window = None

INVERT = -1

H_FoV = cropped_h_fov * (math.pi / 180.0)
V_FoV = cropped_v_fov * (math.pi / 180.0)
unit_vector_map = None
speakers = []

current_retinal_encoded_image = None
# ________________________________________________________________


class Window(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)

        self.view_widget = gl.GLViewWidget(self)
        # Setting the Viewpoint with Azimuth and Elevation
        # http://matlab.izmiran.ru/help/techdoc/visualize/chview3.html
        self.view_widget.setCameraPosition(distance=50,
                                           elevation=-70,
                                           azimuth=95)

        layout = QtGui.QVBoxLayout(self)
        layout.addWidget(self.view_widget)

        # self.setup_grid()
        self.setup_xyz_axis(self.view_widget, 3, 5)

        self.setup_view_box(self.view_widget, 3.0, V_FoV, H_FoV, 15.0)

        head_mesh_data = gl.MeshData.sphere(rows=10, cols=10)
        head = gl.GLMeshItem(meshdata=head_mesh_data, smooth=True, shader='shaded', glOptions='opaque')
        head.translate(0, 0, 0)
        self.view_widget.addItem(head)


    def start(self):
        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()


    def setup_grid(self):
        # NOTE: The axis are according to the right hand rule
        # +z is up
        # +y is right
        # +x is towards you (from the screen)

        x_grid = gl.GLGridItem()
        x_grid.scale(1,1,1)
        self.view_widget.addItem(x_grid)

        y_grid = gl.GLGridItem()
        y_grid.scale(1,1,1)
        y_grid.rotate(90, 0, 1, 0)
        self.view_widget.addItem(y_grid)

        z_grid = gl.GLGridItem()
        z_grid.scale(1,1,1)
        z_grid.rotate(90, 1, 0, 0)
        self.view_widget.addItem(z_grid)


    def setup_xyz_axis(self, view_widget, axis_line_width, axis_length):
        # Line drawing example
        # http://www.pyqtgraph.org/downloads/0.10.0/pyqtgraph-0.10.0-deb/pyqtgraph-0.10.0/examples/GLLinePlotItem.py
        
        # red = x axis
        x_axis_line_data = np.array([[0.0, 0.0, 0.0],[axis_length * INVERT, 0.0, 0.0]])
        x_axis_line_color = (1.0, 0.0, 0.0, 1.0) # color = (r,g,b,a)
        x_axis = gl.GLLinePlotItem(pos=x_axis_line_data, width=axis_line_width, color=x_axis_line_color)
        view_widget.addItem(x_axis)

        # green = y axis
        y_axis_line_data = np.array([[0.0, 0.0, 0.0],[0.0, axis_length, 0.0]])
        y_axis_line_color = (0.0, 1.0, 0.0, 1.0) # color = (r,g,b,a)
        y_axis = gl.GLLinePlotItem(pos=y_axis_line_data, width=axis_line_width, color=y_axis_line_color)
        view_widget.addItem(y_axis)

        # blue = z axis
        z_axis_line_data = np.array([[0.0, 0.0, 0.0],[0.0, 0.0, axis_length]])
        z_axis_line_color = (0.0, 0.0, 1.0, 1.0) # color = (r,g,b,a)
        z_axis = gl.GLLinePlotItem(pos=z_axis_line_data, width=axis_line_width, color=z_axis_line_color)
        view_widget.addItem(z_axis)


    def setup_view_box(self, view_widget, line_width, V_FoV, H_FoV, depth):

        line_color = (1.0, 1.0, 1.0, 1.0) # color = (r,g,b,a)
        x_pos = depth * math.sin(H_FoV / 2.0)
        y_pos = depth * math.sin(V_FoV / 2.0)

        # create the line data
        top_right_line_data = np.array([[0.0, 0.0, 0.0], [x_pos * INVERT, y_pos, depth]])
        top_left_line_data = np.array([[0.0, 0.0, 0.0], [-x_pos * INVERT, y_pos, depth]])
        bottom_right_line_data = np.array([[0.0, 0.0, 0.0], [x_pos * INVERT, -y_pos, depth]])
        bottom_left_line_data = np.array([[0.0, 0.0, 0.0], [-x_pos * INVERT, -y_pos, depth]])
        # connecting lines
        box_data = np.array([[x_pos * INVERT, y_pos, depth], [-x_pos * INVERT, y_pos, depth], [-x_pos * INVERT, -y_pos, depth], [x_pos * INVERT, -y_pos, depth], [x_pos * INVERT, y_pos, depth]])

        # create the lines
        top_right_line = gl.GLLinePlotItem(pos=top_right_line_data, width=line_width, color=line_color)
        top_left_line = gl.GLLinePlotItem(pos=top_left_line_data, width=line_width, color=line_color)
        bottom_right_line = gl.GLLinePlotItem(pos=bottom_right_line_data, width=line_width, color=line_color)
        bottom_left_line = gl.GLLinePlotItem(pos=bottom_left_line_data, width=line_width, color=line_color)
        box = gl.GLLinePlotItem(pos=box_data, width=line_width, color=line_color)
        
        # add the lines to the view
        view_widget.addItem(top_right_line)
        view_widget.addItem(top_left_line)
        view_widget.addItem(bottom_right_line)
        view_widget.addItem(bottom_left_line)
        view_widget.addItem(box)


    def get_position(self, GLMeshItem):
        transform = GLMeshItem.transform()
        x = transform.row(0).w()
        y = transform.row(1).w()
        z = transform.row(2).w()
        return (x, y, z)


    def transform_speaker(self, speaker, x, y, z, size):
        """
        Move and scale ball
        """
        speaker.resetTransform()
        speaker.translate(x, y, z)
        speaker.scale(size, size, size)


    def add_speaker(self, x, y, z, size):
        speaker_mesh_data = gl.MeshData.sphere(rows=10, cols=10)
        speaker = gl.GLMeshItem(meshdata=speaker_mesh_data, smooth=True, shader='viewNormalColor', glOptions='opaque')
        self.transform_speaker(speaker, x, y, z, size)
        self.view_widget.addItem(speaker)
        return speaker


    def update(self):
        if (current_retinal_encoded_image != None):
            retinal_encoded_image_width = len(current_retinal_encoded_image[0])
            retinal_encoded_image_height = len(current_retinal_encoded_image)

            global setup, unit_vector_map, speakers

            if (window != None):
                if not setup:
                    # Generate unit vector map
                    distance_to_near_plane = 0.5 # arbitrary distance
                    pixel_width, pixel_height = ssf_core.calc_pixel_size(distance_to_near_plane,
                                                                         H_FoV,
                                                                         V_FoV,
                                                                         retinal_encoded_image_width,
                                                                         retinal_encoded_image_height)
                    unit_vector_map = ssf_core.generate_unit_vector_map(pixel_width,
                                                                        pixel_height,
                                                                        retinal_encoded_image_width,
                                                                        retinal_encoded_image_height,
                                                                        distance_to_near_plane)

                    for row in xrange(retinal_encoded_image_height):
                        speakers.append([])
                        for column in xrange(retinal_encoded_image_width):
                            speakers[row].append(window.add_speaker(0, 0, 0, 0.5))

                    setup = True

                for row in xrange(retinal_encoded_image_height):
                    for column in xrange(retinal_encoded_image_width):
                        depth = current_retinal_encoded_image[row][column]
                        projected_pixel = ssf_core.projected_pixel(unit_vector_map,
                                                                   column,
                                                                   row,
                                                                   depth * 4)
                        current_speaker = speakers[row][column]
                        window.transform_speaker(current_speaker,
                                                 projected_pixel[0] * INVERT,
                                                 projected_pixel[1] * INVERT,
                                                 projected_pixel[2],
                                                 0.3 + (depth * 0.2))


    def animate(self):

        fps = 60
        
        # update loop every x milliseconds
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(1000 / fps)

        self.start()


def visualization(retinal_encoded_image_imgmsg_format):
    # convert from ROS Image message to OpenCV Image
    # NOTE: An OpenCV Image is essentially a numpy array (super cool)
    retinal_encoded_image = bridge.imgmsg_to_cv2(retinal_encoded_image_imgmsg_format, desired_encoding="32FC1")

    global current_retinal_encoded_image
    current_retinal_encoded_image = retinal_encoded_image


def main():
    rospy.init_node("visualization_3d")

    # Subscribe to the retinal encoded image
    rospy.Subscriber(retinal_encoded_image_topic, Image, visualization)

    global window
    app = QtGui.QApplication([])
    window = Window()
    window.setGeometry(300, 300, 1280, 720)
    window.setWindowTitle('3D Visualization')
    window.show()

    window.animate()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
