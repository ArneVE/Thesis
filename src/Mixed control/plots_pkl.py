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

if sys.argv[1] == '1':
    text = 'MCP'
elif sys.argv[1] == '2':
    text = 'PIP'
elif sys.argv[1] == '3':
    text = 'DIP'

pickle_in = open("r%sED.pickle"%sys.argv[1],"rb")
rED = pickle.load(pickle_in)

pickle_in = open("r%sFP.pickle"%sys.argv[1],"rb")
rFP = pickle.load(pickle_in)

plt.figure(1)
plt.plot(rED[1], np.array(rED[0])*pi*47/4095, 'r-', linewidth = 0.6)
plt.plot(rFP[1], np.array(rFP[0])*pi*60/4095, 'b-', linewidth = 0.6)
leg = plt.legend(('ED', 'FP'))

for legobj in leg.legendHandles:
    legobj.set_linewidth(3.0)

plt.xlabel(r'$\theta_{%s}$ [Deg]'%text)
plt.ylabel('L [mm]')
plt.xticks(np.arange(-40, 101, 20))
plt.yticks(np.arange(-50, 41, 20))
plt.tight_layout()
plt.savefig('r%s.png'%sys.argv[1])
plt.savefig('r%s.eps'%sys.argv[1], format = 'eps')


plt.show()
