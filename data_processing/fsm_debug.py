import matplotlib.pyplot as plt
import numpy as np

import logutils

plt.style.use('dark_background')

data, events = logutils.ingest_log("logs/silsim_datalog.log")

timestamps = data['Simulation']['timestamp'] 

start = 0
stop = 10000

state_mult = 200

plt.figure()
plt.plot(data['Rocket']['timestamp'][start:stop], data['Rocket']['pos_z_enu'][start:stop], color="orange", label="Rocket ENU Altitude [m]")
plt.plot(data['Rocket']['timestamp'][start:stop], data['Rocket']['vel_z_rf'][start:stop], color="blue", label="Rocket RF Z Velocity [m/s]")
plt.plot(data['Rocket']['timestamp'][start:stop], data['Rocket']['accel_z_rf'][start:stop], color="yellow", label="Rocket RF Z Acceleration [m/s^2]")
plt.plot(data['FSM']['timestamp'][start:stop], state_mult * data['FSM']['state'][start:stop], color='r', linewidth=2.0, label="Rocket State Enum")

plt.axhline(0*state_mult, linestyle="--", color='g', linewidth=1.0, label="INIT")
plt.axhline(1*state_mult, linestyle="--", color='g', linewidth=1.0, label="IDLE")
plt.axhline(2*state_mult, linestyle="--", color='g', linewidth=1.0, label="LAUNCH_DETECT")
plt.axhline(3*state_mult, linestyle="--", color='g', linewidth=1.0, label="BOOST")
plt.axhline(4*state_mult, linestyle="--", color='g', linewidth=1.0, label="BURNOUT_DETECT")
plt.axhline(5*state_mult, linestyle="--", color='g', linewidth=1.0, label="COAST")
plt.axhline(6*state_mult, linestyle="--", color='g', linewidth=1.0, label="APOGEE_DETECT")

plt.grid()
plt.legend()
plt.show()
