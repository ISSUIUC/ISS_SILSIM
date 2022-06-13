import matplotlib.pyplot as plt
import numpy as np

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['KalmanFilter']['timestamp'] 

plt.figure()
plt.plot(timestamps, data['KalmanFilter']['Pos'], label="KF Estimated Position [m]")
plt.plot(timestamps, data['KalmanFilter']['Vel'], label="KF Estimated Velocity [m/s]")
plt.plot(timestamps, data['KalmanFilter']['Accel'], label="KF Estimated Accel [m/s^2]")
plt.grid()
plt.legend()
plt.show()
