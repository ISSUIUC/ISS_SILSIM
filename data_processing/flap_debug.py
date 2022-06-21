import matplotlib.pyplot as plt
import numpy as np

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['Simulation']['timestamp'] 

plt.figure()
plt.plot(timestamps, data['Flaps']['real_extension'], label="Flap Real Extension [%]")
plt.plot(timestamps, data['Flaps']['target_extension'], linestyle="--", label="Flap Target Extension [%]")
plt.plot(timestamps, data['Rocket']['total_drag_force_coeff'], label="Rocket Drag Force Coefficient [nd]")
plt.plot(timestamps, data['Rocket']['mach'], label="Rocket Mach Number")
plt.grid()
plt.legend()
plt.show()
