import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import time
import pandas as pd
import numpy as np
import pickle
import math
from math import sin, cos, exp, pi, sqrt, ceil, inf, log
from mpmath import sin, cos, radians, degrees
from scipy.spatial import distance
import scipy
import scipy.interpolate
import sys
import serial
from scipy.constants import g
import matplotlib.pylab as pylab

params = {'legend.fontsize': 23,
      'figure.figsize': (10, 10),
     'axes.labelsize': 40,
     'axes.titlesize': 40,
     'xtick.labelsize': 27,
     'ytick.labelsize': 27}
pylab.rcParams.update(params)

S1 = 9.99546229
S2 = 6.6872175
S3 = 3.92

with open("%s.csv"%sys.argv[1], newline='') as f:
    file = csv.reader(f, delimiter=',', quotechar='"') #['x_cal','z_cal','t_cal','x_meas','z_meas','t_meas','LED_cal','LFP_cal','LED_meas','LFP_meas','wED','wFP','wIO','t_w', 'MCP', 'PIP', 'DIP']

    header = next(file)
    x_cal = np.array(next(file)).astype(float)*100
    z_cal = np.array(next(file)).astype(float)*100
    t_cal = np.array(next(file)).astype(float)
    x_meas = np.array(next(file)).astype(float)*100
    z_meas = np.array(next(file)).astype(float)*100
    t_meas = np.array(next(file)).astype(float)
    LED_cal = np.array(next(file)).astype(float)
    LFP_cal = np.array(next(file)).astype(float)
    LED_meas = np.array(next(file)).astype(float)
    LFP_meas = np.array(next(file)).astype(float)
    wED = np.array(next(file)).astype(float)
    wFP = np.array(next(file)).astype(float)
    wIO = np.array(next(file)).astype(float)
    t_w = np.array(next(file)).astype(float)
    MCP = np.array(next(file)).astype(float)
    PIP = np.array(next(file)).astype(float)
    DIP = np.array(next(file)).astype(float)
    t_angles = np.array(next(file)).astype(float)

t = t_cal[-1]
switch_cal = np.array([np.where(t_cal > t/2)[0][0]])
switch_hyper = np.array([np.where(t_meas > t*0.1)[0][0], np.where(t_meas > t*0.2)[0][0], np.where(t_meas > t*1.1)[0][0], np.where(t_meas > t*1.2)[0][0], np.where(t_meas > t*2.1)[0][0], np.where(t_meas > t*2.2)[0][0]])
switch_w = np.array([np.where(t_w > t/2)[0][0], np.where(t_w > t)[0][0], np.where(t_w > 3*t/2)[0][0], np.where(t_w > 2*t)[0][0], np.where(t_w > 5*t/2)[0][0]])
switch_angles = np.array([np.where(t_angles > t*0.6)[0][0], np.where(t_angles > t)[0][0], np.where(t_angles > t + 0.6*t)[0][0], np.where(t_angles > 2*t)[0][0], np.where(t_angles > 2*t + 0.6*t)[0][0], np.where(t_angles > 3*t)[0][0]])
##switch_angles = np.array([np.where(t_angles > t*0.5)[0][0], np.where(t_angles > t)[0][0], np.where(t_angles > t + 0.5*t)[0][0], np.where(t_angles > 2*t)[0][0], np.where(t_angles > 2*t + 0.5*t)[0][0]])
switch_angles_hyper = np.array([np.where(t_angles > t*0.1)[0][0], np.where(t_angles > t*0.2)[0][0], np.where(t_angles > t*1.1)[0][0], np.where(t_angles > t*1.2)[0][0], np.where(t_angles > t*2.1)[0][0], np.where(t_angles > t*2.2)[0][0]])
#

#
stick_x = np.zeros((MCP.size, 4))
stick_z = np.zeros((MCP.size, 4))
x = []
z = []
for i in range(MCP.size):
    temp1 = S1*cos(radians(MCP[i]))
    temp2 = S2*cos(radians(MCP[i] + PIP[i]))
    temp3 = S3*cos(radians(MCP[i] + PIP[i] + DIP[i]))
    temp4 = S1*sin(radians(MCP[i]))
    temp5 = S2*sin(radians(MCP[i] + PIP[i]))
    temp6 = S3*sin(radians(MCP[i] + PIP[i] + DIP[i]))

    x.append(temp1 + temp2 + temp3 + 6.2234)
    z.append(temp4 + temp5 + temp6 + 5.7345)

    stick_x[i] = np.array([0, temp1, temp1 + temp2, temp1 + temp2 + temp3]) + 6.2234
    stick_z[i] = np.array([0, temp4, temp4 + temp5, temp4 + temp5 + temp6]) + 5.7345

plt.figure()
#plt.plot(x_cal[0:switch_cal[0]], z_cal[0:switch_cal[0]], ':', color = 'black', linewidth = 1.0)

#plt.plot(x, z, '-', color = 'red', linewidth = 0.6)

# Without hyperextension
# plt.plot(x[0:switch_angles[0]], z[0:switch_angles[0]], 'b-', linewidth = 0.6)
# plt.plot(x[switch_angles[1]:switch_angles[2]], z[switch_angles[1]:switch_angles[2]], 'b-', linewidth = 0.6)
# plt.plot(x[switch_angles[3]:switch_angles[4]], z[switch_angles[3]:switch_angles[4]], 'b-', linewidth = 0.6)
#
# plt.plot(x[switch_angles[0]:switch_angles[1]], z[switch_angles[0]:switch_angles[1]], 'r-', linewidth = 0.6)
# plt.plot(x[switch_angles[2]:switch_angles[3]], z[switch_angles[2]:switch_angles[3]], 'r-', linewidth = 0.6)
# plt.plot(x[switch_angles[4]:], z[switch_angles[4]:], 'r-', linewidth = 0.6)

# With hyperextension
plt.plot(x[switch_angles_hyper[0]:switch_angles[0]], z[switch_angles_hyper[0]:switch_angles[0]], 'b-', linewidth = 0.6)
plt.plot(x[switch_angles_hyper[2]:switch_angles[2]], z[switch_angles_hyper[2]:switch_angles[2]], 'b-', linewidth = 0.6)
plt.plot(x[switch_angles_hyper[4]:switch_angles[4]], z[switch_angles_hyper[4]:switch_angles[4]], 'b-', linewidth = 0.6)

plt.plot(x[0:switch_angles_hyper[0]], z[0:switch_angles_hyper[0]], 'r-', linewidth = 0.6)
plt.plot(x[switch_angles[0]:switch_angles_hyper[2]], z[switch_angles[0]:switch_angles_hyper[2]], 'r-', linewidth = 0.6)
plt.plot(x[switch_angles[2]:switch_angles_hyper[4]], z[switch_angles[2]:switch_angles_hyper[4]], 'r-', linewidth = 0.6)
plt.plot(x[switch_angles[4]:switch_angles[5]], z[switch_angles[4]:switch_angles[5]], 'r-', linewidth = 0.6)


plt.plot(stick_x[0], stick_z[0], color = 'black', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.1)[0][0]], stick_z[np.where(t_angles > t*0.1)[0][0]], color = 'blue', marker = 'o', linewidth = 1.5)
#plt.plot(stick_x[np.where(t_angles > t*0.18)[0][0]], stick_z[np.where(t_angles > t*0.18)[0][0]], color = 'blue', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.23)[0][0]], stick_z[np.where(t_angles > t*0.23)[0][0]], color = 'blue', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.31)[0][0]], stick_z[np.where(t_angles > t*0.31)[0][0]], color = 'blue', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.37)[0][0]], stick_z[np.where(t_angles > t*0.37)[0][0]], color = 'blue', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.5)[0][0]], stick_z[np.where(t_angles > t*0.5)[0][0]], color = 'blue', marker = 'o', linewidth = 1.5)

plt.plot(stick_x[np.where(t_angles > t*0.058)[0][0]], stick_z[np.where(t_angles > t*0.058)[0][0]], color = 'red', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.6)[0][0]], stick_z[np.where(t_angles > t*0.6)[0][0]], color = 'red', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.75)[0][0]], stick_z[np.where(t_angles > t*0.75)[0][0]], color = 'red', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.85)[0][0]], stick_z[np.where(t_angles > t*0.85)[0][0]], color = 'red', marker = 'o', linewidth = 1.5)
plt.plot(stick_x[np.where(t_angles > t*0.95)[0][0]], stick_z[np.where(t_angles > t*0.95)[0][0]], color = 'red', marker = 'o', linewidth = 1.5)



plt.xlim((30,-5))
plt.ylim((30,-5))
plt.xticks(np.arange(30, -6, -5))
plt.yticks(np.arange(30, -6, -5))
plt.xlabel('x [cm]')
plt.ylabel('z [cm]')
plt.tight_layout()
plt.savefig('trajectory_%s.png'%sys.argv[1])
plt.savefig('trajectory_%s.eps'%sys.argv[1], format = 'eps')
#
# plt.figure()
# plt.plot(LED_cal*pi*47/4095, LFP_cal*pi*60/4095, 'b-', linewidth = 0.6)
# plt.plot(LED_meas*pi*47/4095, LFP_meas*pi*60/4095, 'r-', linewidth = 0.6)
# leg = plt.legend(('E(F) desired', 'E(F) measured'))
#
# for legobj in leg.legendHandles:
#     legobj.set_linewidth(3.0)
#
# plt.xlim((-30,90))
# plt.ylim((-90,40))
# plt.xlabel('$L_{ED}$ [mm]')
# plt.ylabel('$L_{FP}$ [mm]')
# plt.xticks(np.arange(-30, 91, 20))
# plt.yticks(np.arange(-90, 41, 20))
# plt.tight_layout()
# plt.savefig('L_%s.png'%sys.argv[1])
# plt.savefig('L_%s.eps'%sys.argv[1], format = 'eps')

plt.figure()
plt.plot(t_w[0:switch_w[1]], wED[0:switch_w[1]]*g, 'b-', linewidth = 0.6)
plt.plot(t_w[0:switch_w[1]], wFP[0:switch_w[1]]*g, 'r-', linewidth = 0.6)
plt.plot(t_w[0:switch_w[1]], wIO[0:switch_w[1]]*g, 'g-', linewidth = 0.6)
plt.plot(t_w[switch_w[1]:switch_w[3]] - 15, wED[switch_w[1]:switch_w[3]]*g, 'b-', linewidth = 0.6)
plt.plot(t_w[switch_w[1]:switch_w[3]] - 15, wFP[switch_w[1]:switch_w[3]]*g, 'r-', linewidth = 0.6)
plt.plot(t_w[switch_w[1]:switch_w[3]] - 15, wIO[switch_w[1]:switch_w[3]]*g, 'g-', linewidth = 0.6)
plt.plot(t_w[switch_w[3]:] - 2*15, wED[switch_w[3]:]*g, 'b-', linewidth = 0.6)
plt.plot(t_w[switch_w[3]:] - 2*15, wFP[switch_w[3]:]*g, 'r-', linewidth = 0.6)
plt.plot(t_w[switch_w[3]:] - 2*15, wIO[switch_w[3]:]*g, 'g-', linewidth = 0.6)
plt.xlim((0,15))
plt.xticks(np.arange(0, 16, 1))
plt.yticks(np.arange(0, 51, 5))
leg = plt.legend(('ED', 'FP', 'IO'))

for legobj in leg.legendHandles:
    legobj.set_linewidth(3.0)

plt.xlabel('Time [s]')
plt.ylabel('Force [N]')
plt.tight_layout()
plt.savefig('force_%s.png'%sys.argv[1])
plt.savefig('force_%s.eps'%sys.argv[1], format = 'eps')
#
plt.figure()
#plt.plot(PIP, DIP, 'r-', linewidth = 0.6)

# Without hyperextension
# plt.plot(PIP[0:switch_angles[0]], DIP[0:switch_angles[0]], 'b-', linewidth = 0.6)
# plt.plot(PIP[switch_angles[1]:switch_angles[2]], DIP[switch_angles[1]:switch_angles[2]], 'b-', linewidth = 0.6)
# plt.plot(PIP[switch_angles[3]:switch_angles[4]], DIP[switch_angles[3]:switch_angles[4]], 'b-', linewidth = 0.6)
#
# plt.plot(PIP[switch_angles[0]:switch_angles[1]], DIP[switch_angles[0]:switch_angles[1]], 'r-', linewidth = 0.6)
# plt.plot(PIP[switch_angles[2]:switch_angles[3]], DIP[switch_angles[2]:switch_angles[3]], 'r-', linewidth = 0.6)
# plt.plot(PIP[switch_angles[4]:], DIP[switch_angles[4]:], 'r-', linewidth = 0.6)

# With hyperextension
plt.plot(PIP[switch_angles_hyper[0]:switch_angles[0]], DIP[switch_angles_hyper[0]:switch_angles[0]], 'b-', linewidth = 0.6)
plt.plot(PIP[switch_angles_hyper[2]:switch_angles[2]], DIP[switch_angles_hyper[2]:switch_angles[2]], 'b-', linewidth = 0.6)
plt.plot(PIP[switch_angles_hyper[4]:switch_angles[4]], DIP[switch_angles_hyper[4]:switch_angles[4]], 'b-', linewidth = 0.6)

plt.plot(PIP[0:switch_angles_hyper[0]], DIP[0:switch_angles_hyper[0]], 'r-', linewidth = 0.6)
plt.plot(PIP[switch_angles[0]:switch_angles_hyper[2]], DIP[switch_angles[0]:switch_angles_hyper[2]], 'r-', linewidth = 0.6)
plt.plot(PIP[switch_angles[2]:switch_angles_hyper[4]], DIP[switch_angles[2]:switch_angles_hyper[4]], 'r-', linewidth = 0.6)
plt.plot(PIP[switch_angles[4]:switch_angles[5]], DIP[switch_angles[4]:switch_angles[5]], 'r-', linewidth = 0.6)

plt.xlabel(r'$\theta_{PIP}$ [Deg]')
plt.ylabel(r'$\theta_{DIP}$ [Deg]')
plt.xlim((-80,120))
plt.ylim((-80,120))
plt.xticks(np.arange(-80, 121, 20))
plt.yticks(np.arange(-80, 121, 20))
plt.tight_layout()
plt.savefig('coupling_%s.png'%sys.argv[1])
plt.savefig('coupling_%s.eps'%sys.argv[1], format = 'eps')
#
# plt.figure()
# #plt.plot(MCP, PIP, 'r-', linewidth = 0.6)
#
# # Without hyperextension
# # plt.plot(MCP[0:switch_angles[0]], PIP[0:switch_angles[0]], 'b-', linewidth = 0.6)
# # plt.plot(MCP[switch_angles[1]:switch_angles[2]], PIP[switch_angles[1]:switch_angles[2]], 'b-', linewidth = 0.6)
# # plt.plot(MCP[switch_angles[3]:switch_angles[4]], PIP[switch_angles[3]:switch_angles[4]], 'b-', linewidth = 0.6)
# #
# # plt.plot(MCP[switch_angles[0]:switch_angles[1]], PIP[switch_angles[0]:switch_angles[1]], 'r-', linewidth = 0.6)
# # plt.plot(MCP[switch_angles[2]:switch_angles[3]], PIP[switch_angles[2]:switch_angles[3]], 'r-', linewidth = 0.6)
# # plt.plot(MCP[switch_angles[4]:], PIP[switch_angles[4]:], 'r-', linewidth = 0.6)
#
# # With hyperextension
# plt.plot(MCP[switch_angles_hyper[0]:switch_angles[0]], PIP[switch_angles_hyper[0]:switch_angles[0]], 'b-', linewidth = 0.6)
# plt.plot(MCP[switch_angles_hyper[2]:switch_angles[2]], PIP[switch_angles_hyper[2]:switch_angles[2]], 'b-', linewidth = 0.6)
# plt.plot(MCP[switch_angles_hyper[4]:switch_angles[4]], PIP[switch_angles_hyper[4]:switch_angles[4]], 'b-', linewidth = 0.6)
#
# plt.plot(MCP[0:switch_angles_hyper[0]], PIP[0:switch_angles_hyper[0]], 'r-', linewidth = 0.6)
# plt.plot(MCP[switch_angles[0]:switch_angles_hyper[2]], PIP[switch_angles[0]:switch_angles_hyper[2]], 'r-', linewidth = 0.6)
# plt.plot(MCP[switch_angles[2]:switch_angles_hyper[4]], PIP[switch_angles[2]:switch_angles_hyper[4]], 'r-', linewidth = 0.6)
# plt.plot(MCP[switch_angles[4]:switch_angles[5]], PIP[switch_angles[4]:switch_angles[5]], 'r-', linewidth = 0.6)
#
# plt.xlabel(r'$\theta_{MCP}$ [Deg]')
# plt.ylabel(r'$\theta_{PIP}$ [Deg]')
# plt.xlim((-80,120))
# plt.ylim((-80,120))
# plt.xticks(np.arange(-80, 121, 20))
# plt.yticks(np.arange(-80, 121, 20))
# plt.tight_layout()
# plt.savefig('MCPvPIP_%s.png'%sys.argv[1])
# plt.savefig('MCPvPIP_%s.eps'%sys.argv[1], format = 'eps')
#
# plt.show()

# PIP_plot = []
# wED_plot = []
# wFP_plot = []
# wIO_plot = []
# for i in range(t_w.size):
#     index = np.where(t_angles == t_w[i])
#     if(len(index[0]) == 1):
#         PIP_plot.append(PIP[index[0][0]])
#         wED_plot.append(wED[i])
#         wFP_plot.append(wFP[i])
#         wIO_plot.append(wIO[i])
#
# plt.figure()
# plt.plot(PIP_plot, np.array(wED_plot)*g, 'b-', linewidth = 0.6)
# plt.plot(PIP_plot, np.array(wFP_plot)*g, 'r-', linewidth = 0.6)
# plt.plot(PIP_plot, np.array(wIO_plot)*g, 'g-', linewidth = 0.6)
# plt.xlim((-50,110))
# plt.xticks(np.arange(-50, 111, 20))
# plt.yticks(np.arange(0, 41, 4))
# leg = plt.legend(('ED', 'FP', 'IO'))
#
# for legobj in leg.legendHandles:
#     legobj.set_linewidth(3.0)
#
# plt.xlabel(r'$\theta_{PIP}$ [Deg]')
# plt.ylabel('Force [N]')
# plt.tight_layout()
# plt.savefig('force_vs_PIP_%s.png'%sys.argv[1])
# plt.savefig('force_vs_PIP_%s.eps'%sys.argv[1], format = 'eps')


# """
#     0MCP adjusted file
# """
# PIP = {}
# DIP = {}
# for i in range(6):
#     PIP[i] = []
#     DIP[i] = []
#
# PIP_inter = {}
# DIP_inter = {}
# for i in range(6):
#     PIP_inter[i] = []
#     DIP_inter[i] = []
#
# with open("%s.csv"%sys.argv[1], newline='') as f:
#     file = csv.reader(f, delimiter=',', quotechar='"')
#
#     for row in file:
#         for i in range(6):
#             if row[2*i] == "" or row[2*i+1] == "":
#                 break
#             PIP[i].append(float(row[2*i]))
#             DIP[i].append(float(row[2*i+1]))
#
# for i in range(6):
#     l = len(PIP[i])
#     x = np.arange(0,l)
#     l = 5*l
#     new_x = np.linspace(x.min(), x.max(), l)
#
#     PIP_inter[i] = np.array(scipy.interpolate.interp1d(x, np.array(PIP[i]))(new_x))
#     DIP_inter[i] = np.array(scipy.interpolate.interp1d(x, np.array(DIP[i]))(new_x))

# index_min1 = np.argmin(PIP[0:300])
# index_max1 = 300+np.argmax(PIP[300:3000])
# index_min2 = 3000+np.argmin(PIP[3000:3700])
# index_max2 = 3700+np.argmax(PIP[3700:6200])
# index_min3 = 6200+np.argmin(PIP[6200:6800])
# index_max3 = 6800+np.argmax(PIP[6800:])
#
# switch_angles = np.array([index_max1, np.where(t_angles > t)[0][0], index_max2, np.where(t_angles > 2*t)[0][0], index_max3, np.where(t_angles > 3*t)[0][0]])
# switch_angles_hyper = np.array([index_min1, np.where(t_angles > t*0.2)[0][0], index_min2, np.where(t_angles > t*1.2)[0][0], index_min3, np.where(t_angles > t*2.2)[0][0]])
#
# PIP_inter = {}
# DIP_inter = {}
# for i in range(6):
#     PIP_inter[i] = []
#     DIP_inter[i] = []
#
# #ext
# PIP_inter[0] = np.concatenate((PIP[0:switch_angles_hyper[0]], PIP[switch_angles[0]:switch_angles[1]]))
# DIP_inter[0] = np.concatenate((DIP[0:switch_angles_hyper[0]], DIP[switch_angles[0]:switch_angles[1]]))
# #flex
# PIP_inter[1] = PIP[switch_angles_hyper[0]:switch_angles[0]]
# DIP_inter[1] = DIP[switch_angles_hyper[0]:switch_angles[0]]
# #ext
# PIP_inter[2] = np.concatenate((PIP[switch_angles[1]:switch_angles_hyper[2]], PIP[switch_angles[2]:switch_angles[3]]))
# DIP_inter[2] = np.concatenate((DIP[switch_angles[1]:switch_angles_hyper[2]], DIP[switch_angles[2]:switch_angles[3]]))
# #flex
# PIP_inter[3] = PIP[switch_angles_hyper[2]:switch_angles[2]]
# DIP_inter[3] = DIP[switch_angles_hyper[2]:switch_angles[2]]
# #ext
# PIP_inter[4] = np.concatenate((PIP[switch_angles[3]:switch_angles_hyper[4]], PIP[switch_angles[4]:switch_angles[5]]))
# DIP_inter[4] = np.concatenate((DIP[switch_angles[3]:switch_angles_hyper[4]], DIP[switch_angles[4]:switch_angles[5]]))
# #flex
# PIP_inter[5] = PIP[switch_angles_hyper[4]:switch_angles[4]]
# DIP_inter[5] = DIP[switch_angles_hyper[4]:switch_angles[4]]
#
# diff_x = []
# diff_y = []
# for i in range(3):
#     min_PIP = np.min(PIP_inter[2*i])
#     max_PIP = np.max(PIP_inter[2*i])
#     for j in np.arange(min_PIP, max_PIP, 0.05):
#         index_ext = np.argmin(abs(PIP_inter[2*i] - j))
#         index_flex = np.argmin(abs(PIP_inter[2*i + 1] - j))
#         diff_x.append(PIP_inter[2*i][index_ext])
#         diff_y.append(DIP_inter[2*i+1][index_flex] - DIP_inter[2*i][index_ext])
#
# plt.figure()
# plt.plot(diff_x, diff_y, 'ro', markersize = 0.6)
#
# plt.xlabel(r'$\theta_{PIP}$ [Deg]')
# plt.ylabel(r'$\theta_{DIP,\mathrm{flex}} - \theta_{DIP,\mathrm{ext}}$ [Deg]')
# plt.xlim(-40,100)
# plt.ylim(-10,10)
# plt.xticks(np.arange(-40, 101, 20))
# plt.yticks(np.arange(-10, 11, 2))
# plt.tight_layout()
# plt.savefig('flex-ext_%s.png'%sys.argv[1])
# plt.savefig('flex-ext_%s.eps'%sys.argv[1], format = 'eps')

plt.figure()
plt.plot(t_angles[0:switch_angles[1]], MCP[0:switch_angles[1]], 'r-', linewidth = 0.6)
plt.plot(t_angles[0:switch_angles[1]], PIP[0:switch_angles[1]], 'b-', linewidth = 0.6)
plt.plot(t_angles[0:switch_angles[1]], DIP[0:switch_angles[1]], 'g-', linewidth = 0.6)

plt.plot(t_angles[switch_angles[1]:switch_angles[3]] - 15, MCP[switch_angles[1]:switch_angles[3]], 'r-', linewidth = 0.6)
plt.plot(t_angles[switch_angles[1]:switch_angles[3]] - 15, PIP[switch_angles[1]:switch_angles[3]], 'b-', linewidth = 0.6)
plt.plot(t_angles[switch_angles[1]:switch_angles[3]] - 15, DIP[switch_angles[1]:switch_angles[3]], 'g-', linewidth = 0.6)

plt.plot(t_angles[switch_angles[3]:] - 30, MCP[switch_angles[3]:-1], 'r-', linewidth = 0.6)
plt.plot(t_angles[switch_angles[3]:] - 30, PIP[switch_angles[3]:-1], 'b-', linewidth = 0.6)
plt.plot(t_angles[switch_angles[3]:] - 30, DIP[switch_angles[3]:-1], 'g-', linewidth = 0.6)

#plt.plot(t_angles[np.where(t_angles > t*0.31)[0][0]], PIP[np.where(t_angles > t*0.31)[0][0]], 'ro', linewidth = 2)
#plt.plot(t_angles[np.where(t_angles > t*0.37)[0][0]], PIP[np.where(t_angles > t*0.37)[0][0]], 'ro', linewidth = 2)

plt.xlabel('Time [s]')
plt.ylabel('MCP,PIP,DIP [Deg]')
plt.xlim(0,15)
plt.ylim(-70,111)
plt.yticks(np.arange(-80, 111, 20))
plt.xticks(np.arange(0, 16, 1))
leg = plt.legend(('MCP', 'PIP', 'DIP'))

for legobj in leg.legendHandles:
    legobj.set_linewidth(3.0)

plt.tight_layout()
plt.savefig('time_angles_%s.png'%sys.argv[1])
plt.savefig('time_angles_%s.eps'%sys.argv[1], format = 'eps')
