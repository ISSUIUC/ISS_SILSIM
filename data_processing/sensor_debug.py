import matplotlib.pyplot as plt
import numpy as np

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['Simulation']['timestamp'] 

#plt.figure()
#plt.plot(timestamps, data['Barometer:MS5611_barometer']['baro_altitude'], label="MS5611 Baro Altitude [m]")
#plt.plot(timestamps, data['Rocket']['pos_z_enu'], linestyle="--", label="True Rocket Altitude [m]")
#plt.plot(timestamps, data['Thermometer:MS5611_thermometer']['temperature'], label="MS5611 Baro Temperature [K]")
#plt.grid()
#plt.legend()
#plt.show()

plt.figure()
plt.subplot(611)
plt.plot(data['Accelerometer:LSM9_accel']['timestamp'],
         data['Accelerometer:LSM9_accel']['accel_x_rf'],
         color='g', linewidth=0.8, label="X Accel RF LSM9DS1 [m/s^2]")
plt.plot(data['Rocket']['timestamp'], 
         data['Rocket']['accel_x_rf'], 
         color='r', linestyle='--', linewidth=1.0, label="X Accel RF Truth [m/s^2]")
plt.legend()
plt.subplot(612)
plt.plot(data['Accelerometer:LSM9_accel']['timestamp'],
         data['Accelerometer:LSM9_accel']['accel_y_rf'],
         color='g', linewidth=0.8, label="Y Accel RF LSM9DS1 [m/s^2]")
plt.plot(data['Rocket']['timestamp'], 
         data['Rocket']['accel_y_rf'], 
         color='r', linestyle='--', linewidth=1.0, label="Y Accel RF Truth [m/s^2]")
plt.legend()
plt.subplot(613)
plt.plot(data['Accelerometer:LSM9_accel']['timestamp'],
         data['Accelerometer:LSM9_accel']['accel_z_rf'],
         color='g', linewidth=0.8, label="Z Accel RF LSM9DS1 [m/s^2]")
plt.plot(data['Rocket']['timestamp'], 
         data['Rocket']['accel_z_rf'], 
         color='r', linestyle='--', linewidth=1.0, label="Z Accel RF Truth [m/s^2]")
plt.legend()

plt.subplot(614)
plt.plot(data['Gyroscope:LSM9_gyro']['timestamp'],
         data['Gyroscope:LSM9_gyro']['gyro_x_rf'],
         color='g', linewidth=0.8, label="X Ang. Vel RF LSM9DS1 [rad/sec]")
plt.plot(data['Rocket']['timestamp'],
         data['Rocket']['ang_vel_x_rf'],
         color='r', linestyle='--', linewidth=1.0, 
         label="X Ang. Velocity RF Truth [rad/sec]")
plt.legend()
plt.subplot(615)
plt.plot(data['Gyroscope:LSM9_gyro']['timestamp'],
         data['Gyroscope:LSM9_gyro']['gyro_y_rf'],
         color='g', linewidth=0.8, label="Y Ang. Vel RF LSM9DS1 [rad/sec]")
plt.plot(data['Rocket']['timestamp'],
         data['Rocket']['ang_vel_y_rf'],
         color='r', linestyle='--', linewidth=1.0, 
         label="Y Ang. Velocity RF Truth [rad/sec]")
plt.legend()
plt.subplot(616)
plt.plot(data['Gyroscope:LSM9_gyro']['timestamp'],
         data['Gyroscope:LSM9_gyro']['gyro_z_rf'],
         color='g', linewidth=0.8, label="Z Ang. Vel RF LSM9DS1 [rad/sec]")
plt.plot(data['Rocket']['timestamp'],
         data['Rocket']['ang_vel_z_rf'],
         color='r', linestyle='--', linewidth=1.0, 
         label="Z Ang. Velocity RF Truth [rad/sec]")
plt.legend()

plt.show()
