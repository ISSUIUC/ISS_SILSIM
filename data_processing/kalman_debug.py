import matplotlib.pyplot as plt
import numpy as np
import math

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

data['KalmanFilter']['timestamp'] 

def quat_to_euler(x, y, z, w):
        rad2deg = 180 / np.pi

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1) * rad2deg
     
        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, +1.0)
        pitch = np.arcsin(t2) * rad2deg
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4) * rad2deg
     
        return roll, pitch, yaw # in degrees

# q0 = data['MadgwickAHRS']['q0']
# q1 = data['MadgwickAHRS']['q1']
# q2 = data['MadgwickAHRS']['q2']
# q3 = data['MadgwickAHRS']['q3']

# madgwick_roll, madgwick_pitch, madgwick_yaw = quat_to_euler(q1, q2, q3, q0)
plt.figure()
plt.subplot(311)
plt.plot(data['KalmanFilter']['timestamp'], 
        data['KalmanFilter']['Pos'], label="KF Estimated Position [m]")
plt.plot(data['Rocket']['timestamp'], 
        data['Rocket']['pos_z_enu'], label="Real Alt [m]")        
plt.legend()

plt.subplot(312)
plt.plot(data['KalmanFilter']['timestamp'], 
        data['KalmanFilter']['Vel'], label="KF Estimated Velocity [m/s]")
plt.plot(data['Rocket']['timestamp'], 
        data['Rocket']['vel_z_enu'], label="Real Velocity [m/s]")
plt.legend()

plt.subplot(313)
plt.plot(data['KalmanFilter']['timestamp'], 
        data['KalmanFilter']['Accel'], label="KF Estimated Accel [m/s^2]")
plt.plot(data['Rocket']['timestamp'], 
        data['Rocket']['accel_z_enu'], label="Real Accel [m/s]")
plt.legend()

# plt.subplot(311)
# plt.plot(data['MadgwickAHRS']['timestamp'], madgwick_yaw, label="Estimated Yaw [deg]")
# plt.plot(data['Simulation']['timestamp'], data['Simulation']['yaw'],
#          linestyle='--', color='r', label="Yaw Truth")
# plt.legend()

# plt.subplot(312)
# plt.plot(data['MadgwickAHRS']['timestamp'], madgwick_pitch, label="Estimated Pitch [deg]")
# plt.plot(data['Simulation']['timestamp'], data['Simulation']['pitch'],
#          linestyle='--', color='r', label="Pitch Truth")
# plt.legend()

# plt.subplot(313)
# plt.plot(data['MadgwickAHRS']['timestamp'], madgwick_roll, label="Estimated Roll [deg]")
# plt.plot(data['Simulation']['timestamp'], data['Simulation']['roll'],
#          linestyle='--', color='r', label="Roll Truth")
# plt.legend()

plt.show()
#plt.savefig("kalman_debug.png", dpi=400)
