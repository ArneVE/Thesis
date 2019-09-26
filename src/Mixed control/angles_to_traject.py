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
from mpmath import sin, cos, radians, degrees
from scipy.spatial import distance


class forward_kinematics():
    def __init__(self, MCP, PIP, DIP, t):
        self.S1 = 0.099546229
        self.S2 = 0.066872175
        self.S3 = 0.042911243

        self.MCP = MCP
        self.PIP = PIP
        self.DIP = DIP
        self.t = t

        self.x = []
        self.z = []


    def run(self):
        for i in range(0, len(self.MCP)):
            self.x.append(self.S1*cos(radians(self.MCP[i])) + self.S2*cos(radians(self.MCP[i] + self.PIP[i])) + self.S3*cos(radians(self.MCP[i] + self.PIP[i] + self.DIP[i])))
            self.z.append(self.S1*sin(radians(self.MCP[i])) + self.S2*sin(radians(self.MCP[i] + self.PIP[i])) + self.S3*sin(radians(self.MCP[i] + self.PIP[i] + self.DIP[i])))

        self.x = np.array(self.x) + 0.063141
        self.z = np.array(self.z) + 0.056435


    def plots(self):
        plt.figure(1)
        plt.plot(self.t, self.x, '-', alpha = 1)
        plt.plot(self.t, self.z, '-', alpha = 1)
        plt.legend(('x','z'), loc=2)
        plt.xlabel('Time [s]')
        plt.ylabel('x,z [m]')

        plt.figure(2)
        plt.plot(self.x, self.z, '-', alpha = 1)
        plt.xlabel('x [m]')
        plt.ylabel('z [m]')
        plt.show()

if __name__ == '__main__':
    MCP = [[0.1*i*80/90 for i in np.arange(0, 900)], [0.1*(900 - i)*80/90 for i in np.arange(0, 900)]]
    MCP = sum(MCP, [])
    PIP = [[0.1*i*100/90*80/90 for i in np.arange(0, 900)], [0.1*(900 - i)*100/90*80/90 for i in np.arange(0, 900)]]
    PIP = sum(PIP, [])
    DIP = -6E-11*np.power(PIP, 6) + 8E-09*np.power(PIP, 5) - 3E-08*np.power(PIP, 4) + 2E-05*np.power(PIP, 3) + 0.0014*np.power(PIP, 2) + 0.3904*np.power(PIP, 1) - 0.7545
    t = [i/1800 for i in np.arange(0, 1800)]

    fk = forward_kinematics(MCP, PIP, DIP, t)
    fk.run()
    fk.plots()

    pickle_out = open("x_traject_grasp.pickle","wb")
    pickle.dump(fk.x, pickle_out)

    pickle_out = open("z_traject_grasp.pickle","wb")
    pickle.dump(fk.z, pickle_out)

    pickle_out = open("time_traject_grasp.pickle","wb")
    pickle.dump(fk.t, pickle_out)
