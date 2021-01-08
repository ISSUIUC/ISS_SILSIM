import numpy as np
import matplotlib.animation as animation
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from stl import mesh
import math

plt.style.use('dark_background')


def rotate_mesh(frame):
    rocket_mesh.rotate([0,0,0.5], math.radians(3))
    poly3d.set_verts(rocket_mesh.vectors)

# Create a new plot
fig = plt.figure()
axes = mplot3d.Axes3D(fig)

# Load the STL files and add the vectors to the plot
rocket_mesh = mesh.Mesh.from_file('doge.stl')

poly3d = mplot3d.art3d.Poly3DCollection(rocket_mesh.vectors, edgecolor='k',
                                        facecolor=[0.7, 0.15, 0.15],
                                        linewidths=0.2)
axes.add_collection3d(poly3d)

# Auto scale to the mesh size
scale = rocket_mesh.points.flatten()
axes.auto_scale_xyz(scale, scale, scale)

rocket_animation = animation.FuncAnimation(fig, rotate_mesh, 1,
                                           fargs=(),
                                           interval=0,
                                           blit=False)

# Show the plot to the screen
plt.show()


axes
