import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

plt.style.use('dark_background')

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

roll = []
pitch = []
yaw = []

data_lists = [timestamps, rx, ry, rz, vx, vy, vz, ax, ay, az, fx, fy, fz, roll, pitch, yaw]

with open("sim_data/data.csv") as csvfile:
    csvreader = csv.reader(csvfile, delimiter=",")

    for row in csvreader:
        for i, val in enumerate(row):
            data_lists[i].append(float(val))


plt.figure()
plt.plot(timestamps, rz, label="Altitude")
plt.plot(timestamps, vz, label="Vertical Velocity")
plt.plot(timestamps, az, label="Vertical Acceleration")
plt.plot(timestamps, roll, label="Roll")
plt.plot(timestamps, pitch, label="Pitch")
plt.plot(timestamps, yaw, label="yaw")
plt.grid()
plt.legend()
# plt.show()

fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')
ax.plot(rx,ry,rz,zdir='z')
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax = fig.add_subplot(322)
ax.plot(timestamps, roll, 'r', label="Roll")
ax.grid()
ax.legend()
ax = fig.add_subplot(324)
ax.plot(timestamps, pitch, 'g', label="Pitch")
ax.grid()
ax.legend()
ax = fig.add_subplot(326)
ax.plot(timestamps, yaw, 'b', label="Yaw")
ax.grid()
ax.legend()

plt.show()
