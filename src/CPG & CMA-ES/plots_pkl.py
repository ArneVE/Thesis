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

pickle_in = open("PIP_before.pickle","rb")
PIP_before = pickle.load(pickle_in)

pickle_in = open("DIP_before.pickle","rb")
DIP_before = pickle.load(pickle_in)

pickle_in = open("PIP_after.pickle","rb")
PIP_after = pickle.load(pickle_in)

pickle_in = open("DIP_after.pickle","rb")
DIP_after = pickle.load(pickle_in)

plt.figure(1)
plt.plot(PIP_after, DIP_after, '-', alpha = 1)
plt.plot(PIP_before, DIP_before, '-', alpha = 0.8)
plt.legend(('IPJ coupling after 250 iterations', 'IPJ coupling before 250 iterations'))
plt.xlabel(r'$\theta_{PIP}$ [°]')
plt.ylabel(r'$\theta_{DIP}$ [°]')

plt.show()
