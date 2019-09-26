import matplotlib.pyplot as plt
import os
import csv
import xlsxwriter
import time
import pandas as pd
import numpy as np
import pickle
from scipy.spatial import distance
import math
from math import sin, cos, exp, pi, sqrt, ceil

pickle_off = open("x_traj.pickle","rb")
x_traj = pickle.load(pickle_off)
pickle_off = open("z_traj.pickle","rb")
z_traj = pickle.load(pickle_off)

# Check where the traject ends
#print(len(x_traj))
begin_traj = 270
end_traj = 1612

plt.figure(1)
#plt.plot(x_traj[begin_traj:begin_traj+50], z_traj[begin_traj:begin_traj+50], '.')
#plt.plot(x_traj[end_traj-12:end_traj], z_traj[end_traj-12:end_traj], '.')
plt.plot(x_traj[begin_traj:end_traj], z_traj[begin_traj:end_traj], '.')
plt.ylim(0,0.25)
plt.xlabel("x")
plt.ylabel("z")
plt.show()

print(len(x_traj[begin_traj:end_traj]))
print(x_traj[begin_traj:end_traj].index(min(x_traj[begin_traj:end_traj])))

# Repeat this traject at a certain rate for a certain amount of time
x_traj = np.tile(x_traj[begin_traj:end_traj], 10)
z_traj = np.tile(z_traj[begin_traj:end_traj], 10)
temp = end_traj - begin_traj
time_traj = [i/temp for i in range(len(x_traj))]

plt.figure()
plt.plot(x_traj,z_traj)
plt.xlabel("x [m]")
plt.ylabel("z [m]")
plt.show()

pickle_on = open("x_traject.pickle","wb")
pickle.dump(x_traj, pickle_on)
pickle_on = open("z_traject.pickle","wb")
pickle.dump(z_traj, pickle_on)
pickle_on = open("time_traject.pickle","wb")
pickle.dump(time_traj, pickle_on)
