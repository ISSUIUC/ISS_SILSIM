import matplotlib.pyplot as plt
import numpy as np
import math

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

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

q0 = data['KalmanFilter']['q0']
q1 = data['KalmanFilter']['q1']
q2 = data['KalmanFilter']['q2']
q3 = data['KalmanFilter']['q3']

kf_roll, kf_pitch, kf_yaw = quat_to_euler(q1, q2, q3, q0)

# find when rocket pitches past 30 deg
abort_idx = -1
for idx, angle in enumerate(kf_pitch):
    if angle > 30.0:
        abort_idx = idx
        break

for idx, angle in enumerate(kf_yaw):
    if angle > 30.0 and idx < abort_idx:
        abort_idx = idx
        break

abort_timestamp = data['KalmanFilter']['timestamp'][abort_idx]

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

plt.figure()
plt.subplot(311)
plt.plot(data['Gyroscope:LSM9_gyro']['timestamp'],
         data['Gyroscope:LSM9_gyro']['gyro_x_rf'],
         color='g', linewidth=0.4, label="Gyro X Raw [rad/sec]")
plt.plot(data['Rocket']['timestamp'],
         data['Rocket']['ang_vel_x_rf'],
         color='orange', linestyle='--', linewidth=1.0, 
         label="X Ang. Velocity RF Truth [rad/sec]")
plt.plot(data['KalmanFilter']['timestamp'],
         data['KalmanFilter']['gx_filt'],
         color='r', linestyle='--', linewidth=1.0, 
         label="Gyro X Low-Pass Filtered [rad/sec]")
plt.legend()
plt.subplot(312)
plt.plot(data['Gyroscope:LSM9_gyro']['timestamp'],
         data['Gyroscope:LSM9_gyro']['gyro_y_rf'],
         color='g', linewidth=0.4, label="Gyro Y Raw [rad/sec]")
plt.plot(data['Rocket']['timestamp'],
         data['Rocket']['ang_vel_y_rf'],
         color='orange', linestyle='--', linewidth=1.0, 
         label="Y Ang. Velocity RF Truth [rad/sec]")
plt.plot(data['KalmanFilter']['timestamp'],
         data['KalmanFilter']['gy_filt'],
         color='r', linestyle='--', linewidth=1.0, 
         label="Gyro Y Low-Pass Filtered [rad/sec]")
plt.legend()
plt.subplot(313)
plt.plot(data['Gyroscope:LSM9_gyro']['timestamp'],
         data['Gyroscope:LSM9_gyro']['gyro_z_rf'],
         color='g', linewidth=0.4, label="Gyro Z Raw [rad/sec]")
plt.plot(data['Rocket']['timestamp'],
         data['Rocket']['ang_vel_z_rf'],
         color='orange', linestyle='--', linewidth=1.0, 
         label="Z Ang. Velocity RF Truth [rad/sec]")
plt.plot(data['KalmanFilter']['timestamp'],
         data['KalmanFilter']['gz_filt'],
         color='r', linestyle='--', linewidth=1.0, 
         label="Gyro Z Low-Pass Filtered [rad/sec]")
plt.legend()

plt.figure()
plt.subplot(311)
plt.plot(data['KalmanFilter']['timestamp'], kf_yaw, label="Estimated Yaw [deg]")
plt.plot(data['Simulation']['timestamp'], data['Simulation']['yaw'],
         linestyle='--', color='g', label="Yaw Truth")
plt.axvline(15.0, linestyle='--', color='g', linewidth=0.5, label="Ignition")
plt.axvline(abort_timestamp, linestyle='--', color='r', linewidth=0.8, label="Abort Trigger")
plt.legend()

plt.subplot(312)
plt.plot(data['KalmanFilter']['timestamp'], kf_pitch, label="Estimated Pitch [deg]")
plt.plot(data['Simulation']['timestamp'], data['Simulation']['pitch'],
         linestyle='--', color='g', label="Pitch Truth")
plt.axvline(15.0, linestyle='--', color='g', linewidth=0.5, label="Ignition")
plt.axvline(abort_timestamp, linestyle='--', color='r', linewidth=0.8, label="Abort Trigger")
plt.legend()

plt.subplot(313)
plt.plot(data['KalmanFilter']['timestamp'], kf_roll, label="Estimated Roll [deg]")
plt.plot(data['Simulation']['timestamp'], data['Simulation']['roll'],
         linestyle='--', color='g', label="Roll Truth")
plt.axvline(15.0, linestyle='--', color='g', linewidth=0.5, label="Ignition")
plt.axvline(abort_timestamp, linestyle='--', color='r', linewidth=0.8, label="Abort Trigger")
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
