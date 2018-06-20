# For setup:
# pip install -U PySide
# pip install pyqtgraph
# pip install pyopengl

from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl

import numpy as np
import time

app = QtGui.QApplication([])
view_widget = gl.GLViewWidget()
view_widget.show()
view_widget.setWindowTitle('Example Title')
view_widget.setCameraPosition(distance=50, elevation=30, azimuth=0)

# NOTE: The axis are according to the right hand rule
# +z is up
# +y is right
# +x is towards you (from the screen)


x_grid = gl.GLGridItem()
x_grid.scale(1,1,1)
view_widget.addItem(x_grid)

y_grid = gl.GLGridItem()
y_grid.scale(1,1,1)
y_grid.rotate(90, 0, 1, 0)
view_widget.addItem(y_grid)

z_grid = gl.GLGridItem()
z_grid.scale(1,1,1)
z_grid.rotate(90, 1, 0, 0)
view_widget.addItem(z_grid)

head_mesh_data = gl.MeshData.sphere(rows=10, cols=10)
head = gl.GLMeshItem(meshdata=head_mesh_data, smooth=True, shader='shaded', glOptions='opaque')
head.translate(0, 0, 0)
view_widget.addItem(head)

speaker_mesh_data = gl.MeshData.sphere(rows=10, cols=10)
speaker = gl.GLMeshItem(meshdata=speaker_mesh_data, smooth=True, shader='viewNormalColor', glOptions='opaque')
speaker.translate(5, 5, 5)
speaker.scale(0.5, 0.5, 0.5)
view_widget.addItem(speaker)


## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()