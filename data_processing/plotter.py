import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import csv

import logutils

# Config flags
draw_plots = True

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['Simulation']['timestamp'] 

rx = data['Rocket']['pos_x_enu'] 
ry = data['Rocket']['pos_y_enu'] 
rz = data['Rocket']['pos_z_enu']

vx = data['Rocket']['vel_x_rf']
vy = data['Rocket']['vel_y_rf']
vz = data['Rocket']['vel_z_rf']

aX = data['Rocket']['accel_x_rf']
ay = data['Rocket']['accel_y_rf']
az = data['Rocket']['accel_z_rf']

alpha = np.degrees(data['Rocket']['alpha'])

roll = data['Simulation']['roll']
pitch = data['Simulation']['pitch']
yaw = data['Simulation']['yaw']

rocketX = data['Simulation']['rocket_axis_x']
rocketY = data['Simulation']['rocket_axis_y']
rocketZ = data['Simulation']['rocket_axis_z']

sensorX = data['Accelerometer:LSM9_accel']['accel_x_rf']
sensorY = data['Accelerometer:LSM9_accel']['accel_y_rf']
sensorZ = data['Accelerometer:LSM9_accel']['accel_z_rf']


# print some stats:
apogee = np.max(data['Rocket']['pos_z_enu']) 
print(f"apogee altitude = {apogee} m\n\t\t= {apogee * 3.28} ft")
max_vel = np.max(data['Rocket']['vel_z_enu'])
print(f"max vertical velocity\t= {max_vel} m/s\n\t\t\t= {max_vel * 3.28} ft/s\n\t\t\t= {max_vel / 343} mach")


if draw_plots:

    plt.figure()
    plt.plot(timestamps, data['Rocket']['pos_z_enu'], label="altitude (ENU Frame)")
    plt.plot(timestamps, data['Rocket']['vel_z_enu'], label="vertical velocity (ENU Frame)")
    plt.plot(timestamps, data['Rocket']['accel_z_enu'], label="vertical acceleration (ENU Frame)")

    plt.plot(timestamps, data['KalmanFilter']['Pos'], label="vertical Position (Estimate)")
    plt.plot(timestamps, data['KalmanFilter']['Vel'], label="vertical velocity (Estimate)")
    plt.plot(timestamps, data['KalmanFilter']['Accel'], label="vertical Acceleration (Estimate)")

    plt.plot(timestamps, data['Simulation']['roll'], label="roll")
    plt.plot(timestamps, data['Simulation']['pitch'], label="pitch")
    plt.plot(timestamps, data['Simulation']['yaw'], label="yaw")
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
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.title.set_text("rocket trajectory")

    ax = fig.add_subplot(132, projection='3d')
    ax.plot(rocketX, rocketY, rocketZ, 'r', zdir='z', linewidth=1.0)
    ax.scatter(rocketX[0], rocketY[0], rocketZ[0],color='g', marker='^')
    ax.scatter(rocketX[-1], rocketY[-1], rocketZ[-1], color='r', marker='x')
    ax.axes.set_xlim3d(left=-1.5, right=1.5)
    ax.axes.set_ylim3d(bottom=-1.5, top=1.5)
    ax.axes.set_zlim3d(bottom=-1.5, top=1.5)
    ax.title.set_text("rocket axis evolution")

    ax = fig.add_subplot(333)
    ax.plot(timestamps, roll, 'r', label="roll")
    ax.grid()
    ax.legend()
    ax = fig.add_subplot(336)
    ax.plot(timestamps, pitch, 'g', label="pitch")
    ax.grid()
    ax.legend()
    ax = fig.add_subplot(339)
    ax.plot(timestamps, yaw, 'b', label="yaw")
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

    plt.figure()
    plt.plot(timestamps, alpha, label="Alpha [deg]")
    plt.plot(timestamps, roll, 'r', label="roll")
    plt.plot(timestamps, pitch, 'g', label="pitch")
    plt.plot(timestamps, yaw, 'b', label="yaw")
    plt.grid()
    plt.legend()

    plt.show()
