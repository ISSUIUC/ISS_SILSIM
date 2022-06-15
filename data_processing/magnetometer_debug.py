import matplotlib.pyplot as plt
import numpy as np

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['Simulation']['timestamp'] 

plt.figure()
#plt.plot(timestamps, data['Simulation']['roll'], label="Simulation Roll")
#plt.plot(timestamps, data['Simulation']['pitch'], label="Simulation Pitch")
#plt.plot(timestamps, data['Simulation']['yaw'], label="Simulation Yaw")
plt.plot(timestamps, data['Magnetometer:LSM9_magnetometer']['mag_x_r'], label="Mag X RF")
plt.plot(timestamps, data['Magnetometer:LSM9_magnetometer']['mag_y_rf'], label="Mag Y RF")
plt.plot(timestamps, data['Magnetometer:LSM9_magnetometer']['mag_z_rf'], label="Mag Z RF")
plt.grid()
plt.legend()
plt.show()
