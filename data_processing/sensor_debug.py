import matplotlib.pyplot as plt
import numpy as np

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['Simulation']['timestamp'] 

plt.figure()
plt.plot(timestamps, data['Barometer:MS5611_barometer']['baro_altitude'], label="MS5611 Baro Altitude [m]")
plt.plot(timestamps, data['Rocket']['pos_z_enu'], linestyle="--", label="True Rocket Altitude [m]")
plt.plot(timestamps, data['Thermometer:MS5611_thermometer']['temperature'], label="MS5611 Baro Temperature [K]")
plt.grid()
plt.legend()
plt.show()
