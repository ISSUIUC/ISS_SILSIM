import matplotlib.pyplot as plt
import numpy as np

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['Simulation']['timestamp'] 

plt.figure()
plt.plot(timestamps, data['Atmosphere']['current_wind_magnitude'], label="Magnitude")
plt.plot(timestamps, data['Atmosphere']['current_wind_x'], label="Wind X")
plt.plot(timestamps, data['Atmosphere']['current_wind_y'], label="Wind Y")
plt.plot(timestamps, data['Atmosphere']['current_wind_z'], label="Wind Z")
plt.grid()
plt.legend()
plt.show()
