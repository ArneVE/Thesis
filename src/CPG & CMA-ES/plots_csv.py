import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import xlsxwriter
import time
import pandas as pd
import numpy as np
import pickle
import cma
import math
from math import sin, cos, exp, pi, sqrt, ceil, inf, log
from scipy.spatial import distance

min_val = []
max_val = []
med_val = []

with open('CMAES_output.csv') as f:
    reader = csv.reader(f, delimiter = ',')
    for row in reader:
        min_val.append(float(row[5]))
        med_val.append(float(row[6]))
        max_val.append(float(row[7]))

it = [i for i in range(len(max_val))]
plt.figure(1)
plt.plot(it, med_val, '-')
plt.plot(it, min_val, '-', alpha = 0)
plt.plot(it, max_val, '-', alpha = 0)
plt.fill_between(it, min_val, max_val, color='blue', alpha='0.15')
plt.xlabel('Iterations')
plt.ylabel('f(t)')

plt.show()
