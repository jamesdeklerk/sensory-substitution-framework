# For setup:
# pip install -U PySide
# pip install pyqtgraph
# pip install pyopengl

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import numpy as np
import time
import sys
import math

INVERT_X = -1
count = 0
H_FoV = 70.0 * (math.pi / 180.0)
V_FoV = 45.0 * (math.pi / 180.0)
image_width = 10
image_height = 10

class Window(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)

        self.speakers = []
        self.x_angles = []
        self.y_angles = []

        self.view_widget = gl.GLViewWidget(self)
        # Setting the Viewpoint with Azimuth and Elevation
        # http://matlab.izmiran.ru/help/techdoc/visualize/chview3.html
        self.view_widget.setCameraPosition(distance=50, elevation=-70, azimuth=95)
        self.view_widget.installEventFilter(self)

        layout = QtGui.QVBoxLayout(self)
        layout.addWidget(self.view_widget)

        # self.setup_grid()
        self.setup_xyz_axis(self.view_widget, 3, 5)

        self.setup_view_box(self.view_widget, 3.0, V_FoV, H_FoV, 15.0)

        head_mesh_data = gl.MeshData.sphere(rows=10, cols=10)
        head = gl.GLMeshItem(meshdata=head_mesh_data, smooth=True, shader='shaded', glOptions='opaque')
        head.translate(0, 0, 0)
        self.view_widget.addItem(head)

        for x in xrange(image_width):
            self.speakers.append([])
            for y in xrange(image_height):
                self.speakers[x].append(self.add_speaker(x, y, 5, 0.5))

        # TODO: calculate angles
        # ...
        # self.x_angles
        # self.y_angles
        distance_to_near_plane = 0.5
        pixel_width = self.calc_pixel_size(distance_to_near_plane, H_FoV, image_width)
        pixel_height = self.calc_pixel_size(distance_to_near_plane, V_FoV, image_height)
        for x in xrange(image_width):
            self.x_angles.append(self.calc_angle(distance_to_near_plane, pixel_width, x))
        for y in xrange(image_height):
            self.y_angles.append(self.calc_angle(distance_to_near_plane, pixel_height, y))


    # For pixel width:
    # - FoV = Horizontal FoV
    # - image_size = image width
    # For pixel height
    # - FoV = Vertical FoV
    # - image_size = image height
    def calc_pixel_size(self, distance_to_near_plane, FoV, image_size):
        return ((2.0 * (distance_to_near_plane * 1.0)) * math.tan(FoV / 2.0)) / image_size

    # For vertical
    # - pixel_size = pixel height
    # For horizontal
    # - pixel_size = pixel width
    # pixel_number = i-th pixel from the centre of the image (starting at 1)
    def calc_angle(self, distance_to_near_plane, pixel_size, pixel_number):
        # if odd number of pixels, add half pixel width?
        return math.atan(((pixel_size * pixel_number * 1.0) - (pixel_size / 2.0)) / (distance_to_near_plane * 1.0))

    # Calculate projected position for x or y
    # depth is depth along ray
    def calc_pos_x_or_y(self, depth, angle):
        return depth * math.sin(angle)

    # Always calculated according to the x-axis
    # depth is depth along ray
    def calc_pos_z(self, depth, angle):
        return depth * math.cos(angle)

    def calc_projected_xyz(self, depth, x_th_pixel, y_th_pixel):
        x = self.calc_pos_x_or_y(depth, self.x_angles[x_th_pixel - 1])
        y = self.calc_pos_x_or_y(depth, self.y_angles[y_th_pixel - 1])
        z = 5
        return (x, y, z)

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
        x_axis_line_data = np.array([[0.0, 0.0, 0.0],[axis_length * INVERT_X, 0.0, 0.0]])
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
        top_right_line_data = np.array([[0.0, 0.0, 0.0], [x_pos * INVERT_X, y_pos, depth]])
        top_left_line_data = np.array([[0.0, 0.0, 0.0], [-x_pos * INVERT_X, y_pos, depth]])
        bottom_right_line_data = np.array([[0.0, 0.0, 0.0], [x_pos * INVERT_X, -y_pos, depth]])
        bottom_left_line_data = np.array([[0.0, 0.0, 0.0], [-x_pos * INVERT_X, -y_pos, depth]])
        # connecting lines
        box_data = np.array([[x_pos * INVERT_X, y_pos, depth], [-x_pos * INVERT_X, y_pos, depth], [-x_pos * INVERT_X, -y_pos, depth], [x_pos * INVERT_X, -y_pos, depth], [x_pos * INVERT_X, y_pos, depth]])

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

    def update_speaker_using_keys(self, speaker, x, y, z):
        position = self.get_position(speaker)
        self.transform_speaker(speaker, position[0] + x, position[1] + y, position[2] + z, 1.5)
        print self.get_position(speaker)

    def update(self):
        global count
        count = count + 0.04

        num_x = len(self.speakers)
        num_y = len(self.speakers[0])

        for x in xrange(num_x):
            for y in xrange(num_y):
                x_value = x - (num_x / 2.0)
                y_value = y - (num_y / 2.0)
                z_value = math.cos(((y - (num_y / 2.0)) / 2.0) + count) + math.cos(((x - (num_x / 2.0)) / 2.0) + count)
                # normalize z
                z_value = (z_value + 2.0) / 4.0
                z_value = z_value * 15.0
                self.transform_speaker(self.speakers[x][y], x_value * INVERT_X, y_value, z_value, 0.5)
        

    def animate(self):

        fps = 60
        
        # update loop every x milliseconds
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(1000 / fps)

        self.start()

    def eventFilter(self, widget, event):
        if (event.type() == QtCore.QEvent.KeyPress and
            widget is self.view_widget):
            key = event.key()

            MOVE_DISTANCE = 0.5

            # move up
            if key == QtCore.Qt.Key_W:
                self.update_speaker_using_keys(self.speakers[0][0], 0, 0, MOVE_DISTANCE)
            # move down
            if key == QtCore.Qt.Key_S:
                self.update_speaker_using_keys(self.speakers[0][0], 0, 0, -MOVE_DISTANCE)
            # move left
            if key == QtCore.Qt.Key_A:
                self.update_speaker_using_keys(self.speakers[0][0], -MOVE_DISTANCE * INVERT_X, 0, 0)
            # move right
            if key == QtCore.Qt.Key_D:
                self.update_speaker_using_keys(self.speakers[0][0], MOVE_DISTANCE * INVERT_X, 0, 0)
            # move backwards
            if key == QtCore.Qt.Key_F:
                self.update_speaker_using_keys(self.speakers[0][0], MOVE_DISTANCE, 0, 0)
            # move forwards (into screen)
            if key == QtCore.Qt.Key_R:
                self.update_speaker_using_keys(self.speakers[0][0], -MOVE_DISTANCE, 0, 0)
            
        return QtGui.QWidget.eventFilter(self, widget, event)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    app = QtGui.QApplication([])
    window = Window()
    window.setGeometry(300, 300, 1280, 720)
    window.setWindowTitle('Example Title')
    window.show()

    window.animate()
