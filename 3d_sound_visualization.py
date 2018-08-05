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

class Window(QtGui.QWidget):

    def __init__(self):
        QtGui.QWidget.__init__(self)

        self.speakers = []

        self.view_widget = gl.GLViewWidget(self)
        self.view_widget.setCameraPosition(distance=50, elevation=30, azimuth=0)
        self.view_widget.installEventFilter(self)

        layout = QtGui.QVBoxLayout(self)
        layout.addWidget(self.view_widget)

        self.setup_grid()

        head_mesh_data = gl.MeshData.sphere(rows=10, cols=10)
        head = gl.GLMeshItem(meshdata=head_mesh_data, smooth=True, shader='shaded', glOptions='opaque')
        head.translate(0, 0, 0)
        self.view_widget.addItem(head)

        num_x = 10
        num_y = 10
        count = 0
        for x in xrange(num_x):
            for y in xrange(num_y):
                self.speakers.append(self.add_speaker(x, y, 5, 0.5))
                count = count + 1

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

    def update(self, speaker, x, y, z):
        position = self.get_position(speaker)
        self.transform_speaker(speaker, position[0] + x, position[1] + y, position[2] + z, 1.5)
        print self.get_position(speaker)

    def animate(self):
        # timer = QtCore.QTimer()
        # timer.timeout.connect(self.update)
        # timer.start(100)
        self.start()

    def eventFilter(self, widget, event):
        if (event.type() == QtCore.QEvent.KeyPress and
            widget is self.view_widget):
            key = event.key()

            MOVE_DISTANCE = 0.5

            # move up
            if key == QtCore.Qt.Key_W:
                self.update(self.speakers[0], 0, 0, MOVE_DISTANCE)
            # move down
            if key == QtCore.Qt.Key_S:
                self.update(self.speakers[0], 0, 0, -MOVE_DISTANCE)
            # move left
            if key == QtCore.Qt.Key_A:
                self.update(self.speakers[0], 0, -MOVE_DISTANCE, 0)
            # move right
            if key == QtCore.Qt.Key_D:
                self.update(self.speakers[0], 0, MOVE_DISTANCE, 0)
            # move backwards
            if key == QtCore.Qt.Key_F:
                self.update(self.speakers[0], MOVE_DISTANCE, 0, 0)
            # move forwards (into screen)
            if key == QtCore.Qt.Key_R:
                self.update(self.speakers[0], -MOVE_DISTANCE, 0, 0)
            
            
        return QtGui.QWidget.eventFilter(self, widget, event)

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    app = QtGui.QApplication([])
    window = Window()
    window.setGeometry(300, 300, 1280, 720)
    window.setWindowTitle('Example Title')
    window.show()

    window.animate()
