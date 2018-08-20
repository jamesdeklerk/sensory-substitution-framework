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
H_FoV = 63.4 * (math.pi / 180.0)
V_FoV = 40.4 * (math.pi / 180.0)
image_width = 30
image_height = 30

global_x_pos = (image_width - 1)
global_y_pos = (image_height - 1)
global_z_pos = 10.0
global_direction = "down"


class Window(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)

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

        # Generate unit vector map
        distance_to_near_plane = 0.5
        pixel_width = self.calc_pixel_size(distance_to_near_plane, H_FoV, image_width)
        pixel_height = self.calc_pixel_size(distance_to_near_plane, V_FoV, image_height)
        self.unit_vector_map = self.generate_unit_vector_map(pixel_width, pixel_height, image_width, image_height, distance_to_near_plane)

        self.speaker_moveing_in_square = self.add_speaker(global_x_pos, global_y_pos, global_z_pos, 0.5)


    # For pixel width:
    # - FoV = Horizontal FoV
    # - num_pixels = image width
    # For pixel height
    # - FoV = Vertical FoV
    # - num_pixels = image height
    def calc_pixel_size(self, distance_to_near_plane, FoV, num_pixels):
        return ((2.0 * (distance_to_near_plane * 1.0)) * math.tan(FoV / 2.0)) / num_pixels

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

    def calc_unit_vector(self, pixel_width, pixel_height, x_th_pixel_from_centre, y_th_pixel_from_centre, num_x_pixels, num_y_pixels, distance_to_near_plane):
        image_width_odd = not ((num_x_pixels % 2.0) == 0.0)
        if image_width_odd:
            x_position = (x_th_pixel_from_centre * pixel_width)
        else:
            if (x_th_pixel_from_centre > 0): 
                # if the x-th pixel from the centre is positive, minus half the pixel size
                x_position = (x_th_pixel_from_centre * pixel_width) - (pixel_width / 2.0)
            else:
                # if the x-th pixel from the centre is negative, add half the pixel size
                x_position = (x_th_pixel_from_centre * pixel_width) + (pixel_width / 2.0)

        image_height_odd = not ((num_y_pixels % 2.0) == 0.0)
        if image_height_odd:
            y_position = (y_th_pixel_from_centre * pixel_height)
        else:
            if (y_th_pixel_from_centre > 0): 
                # if the y-th pixel from the centre is positive, minus half the pixel size
                y_position = (y_th_pixel_from_centre * pixel_height) - (pixel_height / 2.0)
            else:
                # if the y-th pixel from the centre is negative, add half the pixel size
                y_position = (y_th_pixel_from_centre * pixel_height) + (pixel_height / 2.0)
        
        vector_length = math.sqrt((x_position ** 2.0) + (y_position ** 2.0) + (distance_to_near_plane ** 2.0))

        return ((x_position / vector_length), (y_position / vector_length), (distance_to_near_plane / vector_length))

    # TODO
    def generate_unit_vector_map(self, pixel_size_x, pixel_size_y, num_x_pixels, num_y_pixels, distance_to_near_plane):
        image_width_odd = not ((num_x_pixels % 2.0) == 0.0)
        image_height_odd = not ((num_y_pixels % 2.0) == 0.0)
        half_width_floored = math.floor(num_x_pixels / 2)
        half_height_floored = math.floor(num_y_pixels / 2)

        unit_vector_map = []
        for x_th_pixel in xrange(num_x_pixels):
            unit_vector_map.append([])
            for y_th_pixel in xrange(num_y_pixels):
                # center x
                x_th_pixel_from_centre = x_th_pixel - half_width_floored
                if x_th_pixel_from_centre >= 0 and (not image_width_odd):
                    # this skips 0 for even width images
                    x_th_pixel_from_centre = (x_th_pixel - half_width_floored) + 1
                
                # center y
                y_th_pixel_from_centre = y_th_pixel - half_height_floored
                if y_th_pixel_from_centre >= 0 and (not image_height_odd):
                    # this skips 0 for even height images
                    y_th_pixel_from_centre = (y_th_pixel - half_height_floored) + 1
                # Invert y because top of image
                # y_th_pixel_from_centre = -y_th_pixel_from_centre
                
                unit_vector_map[x_th_pixel].append(self.calc_unit_vector(pixel_size_x, pixel_size_y, x_th_pixel_from_centre, y_th_pixel_from_centre, num_x_pixels, num_y_pixels, distance_to_near_plane))
        return unit_vector_map

    def projected_pixel(self, x_th_pixel, y_th_pixel, depth):
        unit_vector = self.unit_vector_map[x_th_pixel][y_th_pixel] # get unit vector
        projected_pixel = (unit_vector[0] * depth, unit_vector[1] * depth, unit_vector[2] * depth)
        # return (x, y, z)
        return projected_pixel

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
        global count, global_x_pos, global_y_pos, global_z_pos, global_direction
        count = count + 1

        # move in square
        
        # only update every 5 frames
        if (count > 5):
            count = 0

            if global_direction == "right":
                global_x_pos = global_x_pos + 1
                if global_x_pos > (image_width - 1):
                    global_x_pos = (image_width - 1)
                    global_direction = "down"
            elif global_direction == "down":
                global_y_pos = global_y_pos - 1
                if global_y_pos < 0:
                    global_y_pos = 0
                    global_direction = "left"
            elif global_direction == "left":
                global_x_pos = global_x_pos - 1
                if global_x_pos < 0:
                    global_x_pos = 0
                    global_direction = "up"
            elif global_direction == "up":
                global_y_pos = global_y_pos + 1
                if global_y_pos > (image_height - 1):
                    global_y_pos = (image_height - 1)
                    global_direction = "right"

            # move
            projected_pixel = self.projected_pixel(global_x_pos, global_y_pos, global_z_pos)
            self.transform_speaker(self.speaker_moveing_in_square, projected_pixel[0] * INVERT_X, projected_pixel[1], projected_pixel[2], 0.5)
        

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

            global global_z_pos
            z_inc = 1

            # move up
            if key == QtCore.Qt.Key_W:
                global_z_pos = global_z_pos + z_inc
            # move down
            if key == QtCore.Qt.Key_S:
                global_z_pos = global_z_pos - z_inc
            
        return QtGui.QWidget.eventFilter(self, widget, event)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    app = QtGui.QApplication([])
    window = Window()
    window.setGeometry(300, 300, 1280, 720)
    window.setWindowTitle('Example Title')
    window.show()

    window.animate()
