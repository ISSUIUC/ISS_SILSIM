###############################################################################
# ISS SILSIM - RASAero Rocket Coefficient Analysis
#
# Created 04/05/2022
# 
# Authors:
# Ayberk Yaraneri
#
###############################################################################

import matplotlib.pyplot as plt
from matplotlib import cm
import numpy as np


data_array = np.genfromtxt("RASAero_Intrepid_5800_mk6.csv", delimiter=",", skip_header=1)

mach_cutoff = 3.0

num_machs = int(mach_cutoff / 0.01) 
num_alphas = 16
num_protubs = 6

machs = np.arange(0.01, mach_cutoff+0.01, 0.01)
alphas = np.arange(0.0, 16.0, 1.0)
protubs = np.arange(0.0, 1.2, 0.2)

cd_poweroff = np.zeros((num_machs, num_alphas, num_protubs))
cd_poweron  = np.zeros((num_machs, num_alphas, num_protubs))
ca_poweroff = np.zeros((num_machs, num_alphas, num_protubs))
ca_poweron  = np.zeros((num_machs, num_alphas, num_protubs))
cn_total    = np.zeros((num_machs, num_alphas, num_protubs))
cp_total    = np.zeros((num_machs, num_alphas, num_protubs))

idx = 0

while data_array[idx,0] <= mach_cutoff:

    data = data_array[idx,:]

    mach = data[0]
    mach_idx = round(mach * 100) - 1

    alpha = data[1]
    alpha_idx = round(alpha)

    protub = data[2]
    protub_idx = round(protub / 0.2)

    cd_poweroff[mach_idx, alpha_idx, protub_idx] = data[3]
    cd_poweron[mach_idx, alpha_idx, protub_idx]  = data[4]
    ca_poweroff[mach_idx, alpha_idx, protub_idx] = data[5]
    ca_poweron[mach_idx, alpha_idx, protub_idx]  = data[6]
    cn_total[mach_idx, alpha_idx, protub_idx]    = data[7]
    cp_total[mach_idx, alpha_idx, protub_idx]    = data[8]

    idx += 1

X,Y = np.meshgrid(machs, alphas)
Z = ca_poweroff[:,:,0].transpose()
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot_surface(X,Y,Z, cmap=cm.inferno, linewidth=5.0, rstride=1, cstride=1, 
                antialiased=False)
plt.xlim(0, 3.0)
plt.title("Power-off Axial Force Coefficient")
plt.xlabel("Mach Number")
plt.ylabel("Alpha")
plt.savefig("axial_force_coeff.png", dpi=800)

X,Y = np.meshgrid(machs, alphas)
Z = cn_total[:,:,0].transpose()
fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot_surface(X,Y,Z, cmap=cm.inferno, linewidth=5.0, rstride=1, cstride=1, 
                antialiased=False)
plt.xlim(0, 3.0)
plt.title("Total Normal Force Coefficient")
plt.xlabel("Mach Number")
plt.ylabel("Alpha")
plt.savefig("normal_force_coeff.png", dpi=800)
