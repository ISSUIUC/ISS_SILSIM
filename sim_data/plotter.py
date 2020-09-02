import matplotlib.pyplot as plt
import csv

timestamps = []

rx = []
ry = []
rz = []

vx = []
vy = []
vz = []

ax = []
ay = []
az = []

fx = []
fy = []
fz = []

data_lists = [timestamps, rx, ry, rz, vx, vy, vz, ax, ay, az, fx, fy, fz]

with open("sim_data/data.csv") as csvfile:
    csvreader = csv.reader(csvfile, delimiter=",")

    for row in csvreader:
        for i, val in enumerate(row):
            data_lists[i].append(float(val))


plt.figure()

plt.plot(timestamps, rz, label="Altitude")
plt.plot(timestamps, vz, label="Vertical Velocity")
plt.plot(timestamps, az, label="Vertical Acceleration")
plt.grid()
plt.legend()
plt.show()
