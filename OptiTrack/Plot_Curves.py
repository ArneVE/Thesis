import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import time
import pandas as pd
import numpy as np
import math

data = pd.read_csv('raw_data.csv', delimiter = ';')
nPoints = data['Time'].values.size

PIP_angle = []
DIP_angle = []

for i in range(0, nPoints):
    vPPx = data['PP_data1x'].values[i] - data['PP_data2x'].values[i]
    vPPz = data['PP_data1z'].values[i] - data['PP_data2z'].values[i]
    vMPx = data['MP_data1x'].values[i] - data['MP_data2x'].values[i]
    vMPz = data['MP_data1z'].values[i] - data['MP_data2z'].values[i]
    vDPx = data['DP_data1x'].values[i] - data['DP_data2x'].values[i]
    vDPz = data['DP_data1z'].values[i] - data['DP_data2z'].values[i]
    vPP = (vPPx, vPPz)
    vMP = (vMPx, vMPz)
    vDP = (vDPx, vDPz)
    PIP_angle.append(math.degrees(math.atan2(vPPx*vMPz - vPPz*vMPx, vPPx*vMPx + vPPz*vMPz)))
    DIP_angle.append(math.degrees(math.atan2(vMPx*vDPz - vMPz*vDPx, vMPx*vDPx + vMPz*vDPz)))


columns = zip(PIP_angle, DIP_angle)
with open('angles.csv', "w") as f:
    writer = csv.writer(f)
    writer.writerow(['PIP_angle,DIP_angle'])
    for row in columns:
        writer.writerow(row)

plt.plot(PIP_angle, DIP_angle)
plt.title("PIP-DIP coupling")
plt.xlabel("PIP angle [degrees]")
plt.ylabel("DIP angle [degrees]")
plt.show()
