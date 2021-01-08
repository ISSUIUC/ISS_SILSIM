import numpy as np
import matplotlib.animation as animation
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from stl import mesh
import math
import csv
import copy

plt.style.use('dark_background')

################################## Fetch Data ##################################

timestamps = []

rx = []
ry = []
rz = []

vx = []
vy = []
vz = []

ax = []
ay = []
az = []

fx = []
fy = []
fz = []

q0 = []
q1 = []
q2 = []
q3 = []

roll = []
pitch = []
yaw = []

rocketX = []
rocketY = []
rocketZ = []

data_lists = [timestamps, rx, ry, rz, vx, vy, vz, ax, ay, az, fx, fy, fz,
              q0, q1, q2, q3, roll, pitch, yaw, rocketX, rocketY, rocketZ]

with open("sim_data/data.csv") as csvfile:
    csvreader = csv.reader(csvfile, delimiter=",")

    for row in csvreader:
        for i, val in enumerate(row):
            data_lists[i].append(float(val))

nFrames = len(timestamps)

################################# 3D Animation #################################

def rotate_mesh(frame, quatrns):
    rocket_mesh = copy.deepcopy(original_mesh);
    angle = 2 * math.acos(quatrns[frame, 0])
    x = quatrns[frame,1]
    y = quatrns[frame,2]
    z = quatrns[frame,3]
    rocket_mesh.rotate([x, y, z], angle)
    poly3d.set_verts(rocket_mesh.vectors)

# Create a new plot
fig = plt.figure()
axes = mplot3d.Axes3D(fig)

# Load the STL files and add the vectors to the plot
original_mesh = mesh.Mesh.from_file('sim_data/doge.stl')

poly3d = mplot3d.art3d.Poly3DCollection(original_mesh.vectors, edgecolor='k',
                                        facecolor=[0.7, 0.15, 0.15],
                                        linewidths=0.2)
axes.add_collection3d(poly3d)

# Auto scale to the mesh size
scale = original_mesh.points.flatten()
axes.auto_scale_xyz(scale, scale, scale)

quatrns = np.array([q0, q1, q2, q3]).T

rocket_animation = animation.FuncAnimation(fig, rotate_mesh,
                                           frames=np.arange(0,nFrames,10),
                                           fargs=([quatrns]),
                                           interval=0.1,
                                           blit=False)

# Show the plot to the screen
# plt.show()

videowriter = animation.FFMpegWriter(fps=30)
rocket_animation.save("sim_data/animation.mp4", writer=videowriter)
