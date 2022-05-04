import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv

# Config flags
draw_plots = True

plt.style.use('dark_background')

timestamps = []

rx = []
ry = []
rz = []

vx = []
vy = []
vz = []

aX = []
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

sensorX = []
sensorY = []
sensorZ = []

data_lists = [timestamps, rx, ry, rz, vx, vy, vz, aX, ay, az, fx, fy, fz,
              q0, q1, q2, q3, roll, pitch, yaw, rocketX, rocketY, rocketZ,
              sensorX, sensorY, sensorZ]

with open("sim_data/data.csv") as csvfile:
    csvreader = csv.reader(csvfile, delimiter=",")

    for row in csvreader:
        for i, val in enumerate(row):
            data_lists[i].append(float(val))

# Print some stats:
apogee = np.max(rz) 
print(f"Apogee Altitude = {apogee} m\n\t\t= {apogee * 3.28} ft")
max_vel = np.max(vz)
print(f"Max Vertical Velocity\t= {max_vel} m/s\n\t\t\t= {max_vel * 3.28} ft/s\n\t\t\t= {max_vel / 343} mach")

# KF plotting
import pandas as pd

kalman_df = pd.read_csv("Kalman.csv")
kx = kalman_df["Pos"]
kv = kalman_df["Vel"]
ka = kalman_df["Accel"]
pk00 = kalman_df["invertboi_00"]
pk01 = kalman_df["invertboi_01"]
pk10 = kalman_df["invertboi_10"]
pk11 = kalman_df["invertboi_11"]

apogee_df = pd.read_csv("apogee.csv")

if draw_plots:

    plt.figure()
    plt.plot(timestamps, rz, label="Altitude")
    plt.plot(timestamps, vz, label="Vertical Velocity")
    plt.plot(timestamps, az, label="Vertical Acceleration")
    # plt.plot(timestamps, roll, label="Roll")
    # plt.plot(timestamps, pitch, label="Pitch")
    # plt.plot(timestamps, yaw, label="yaw")
    
    # KF Plotting
    timestamps_kf = np.arange(0.003,(len(kalman_df)) * 0.006, 0.006)
    # plt.plot(timestamps_kf,pk00, label="invertboi00")
    # plt.plot(timestamps_kf,pk01, label="invertboi01")
    # plt.plot(timestamps_kf,pk10,label="invertboi10")
    # plt.plot(timestamps_kf,pk11, label="invertboi11")
<<<<<<< HEAD
    plt.plot(timestamps_kf, apogee_df["apogee"], label="RK4 Apogee")
=======
>>>>>>> fea03379476a74ed4bc16665df3ab7ce6d842895
    plt.plot(timestamps_kf, kx, label="KF Altitude")
    plt.plot(timestamps_kf, kv, label="KF Velocity")
    plt.plot(timestamps_kf, ka, label="KF Acceleration")
    # plt.ylim(-100, 12000)
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
    ax.axes.set_zlim3d(bottom=0.00001, top=15000)
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

    fig = plt.figure()
    ax = fig.add_subplot(311)
    plt.plot(timestamps, aX, color='r', linewidth=2.0, label="x accel truth")
    plt.plot(timestamps, sensorX, color='g', label="x accel sensor output")
    plt.grid()
    plt.legend()
    ax = fig.add_subplot(312)
    plt.plot(timestamps, ay, color='r', linewidth=2.0, label="y accel truth")
    plt.plot(timestamps, sensorY, color='g', label="y accel sensor output")
    plt.grid()
    plt.legend()
    ax = fig.add_subplot(313)
    plt.plot(timestamps, az, color='r', linewidth=2.0, label="z accel truth")
    plt.plot(timestamps, sensorZ, color='g', label="z accel sensor output")
    plt.grid()
    plt.legend()

    plt.show()
