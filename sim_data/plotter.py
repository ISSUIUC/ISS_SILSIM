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


plt.figure()
plt.plot(timestamps, rz, label="Altitude")
plt.plot(timestamps, vz, label="Vertical Velocity")
plt.plot(timestamps, az, label="Vertical Acceleration")
plt.plot(timestamps, roll, label="Roll")
plt.plot(timestamps, pitch, label="Pitch")
plt.plot(timestamps, yaw, label="yaw")
plt.grid()
plt.legend()

fig = plt.figure()
fig.subplots_adjust(left=0.000, top=0.980, bottom=0.045, right=0.980, wspace=0.100, hspace=0.100)

ax = fig.add_subplot(131, projection='3d')
ax.plot(rx,ry,rz,'b', zdir='z')
ax.scatter(rx[0],ry[0],rz[0],color='g', marker='^')
ax.scatter(rx[-1],ry[-1],rz[-1], color='r', marker='x')
ax.axes.set_xlim3d(left=-3000, right=3000)
ax.axes.set_ylim3d(bottom=-3000, top=3000)
ax.axes.set_zlim3d(bottom=0.00001, top=3000)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.title.set_text("Rocket Trajectory")

ax = fig.add_subplot(132, projection='3d')
ax.plot(rocketX, rocketY, rocketZ, 'r', zdir='z', linewidth=1.0)
ax.scatter(rocketX[0], rocketY[0], rocketZ[0],color='g', marker='^')
ax.scatter(rocketX[-1], rocketY[-1], rocketZ[-1], color='r', marker='x')
ax.axes.set_xlim3d(left=-1.5, right=1.5)
ax.axes.set_ylim3d(bottom=-1.5, top=1.5)
ax.axes.set_zlim3d(bottom=-1.5, top=1.5)
ax.title.set_text("Rocket Axis Evolution")

ax = fig.add_subplot(333)
ax.plot(timestamps, roll, 'r', label="Roll")
ax.grid()
ax.legend()
ax = fig.add_subplot(336)
ax.plot(timestamps, pitch, 'g', label="Pitch")
ax.grid()
ax.legend()
ax = fig.add_subplot(339)
ax.plot(timestamps, yaw, 'b', label="Yaw")
ax.grid()
ax.legend()

plt.show()
