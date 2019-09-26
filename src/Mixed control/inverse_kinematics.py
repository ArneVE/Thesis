import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import sys
import xlsxwriter
import time
import pandas as pd
import numpy as np
import pickle
import cma
import math
from math import sin, cos, exp, pi, sqrt, ceil, inf, log
from scipy.spatial import distance
from scipy import optimize
from scipy.signal import savgol_filter
import matplotlib.pylab as pylab

from traject_to_angles import traject_to_angles

class inverse_kinematics():
    def __init__(self, x, z, t):
        self.x = x
        self.z = z
        self.t = np.array(t)
        if (max(t) > 2):
            self.lb = np.where(self.t > 1)[0][0]
            self.ub = np.where(self.t > 2)[0][0]
            stop = np.where(self.t > 3)[0][0]
        else:
            self.lb = 0
            self.ub = t.size
            stop = self.ub
        self.x = self.x[0:stop+1]
        self.z = self.z[0:stop+1]
        self.t = self.t[0:stop+1]
        self.dLED = []
        self.dLFP = []

        # For hyperextension calculation
        self.MCP = x
        self.PIP = z

    def momentarm(self, r):
        L = np.array(r)[0]
        theta = np.array(r)[1]
        poly = np.polyfit(theta, L, 4)
        f = np.poly1d(poly)

        # theta_new = np.linspace(-50, 110, 1000)
        # LED_new = f(theta_new)
        #
        # plt.figure()
        # plt.plot(theta,L)
        # plt.plot(theta_new, LED_new)
        # plt.show()

        return f


    def load_data(self):
        pickle_off = open("r1ED.pickle","rb")
        r1ED = pickle.load(pickle_off)
        self.f_r1ED = self.momentarm(r1ED)

        pickle_off = open("r1FP.pickle","rb")
        r1FP = pickle.load(pickle_off)
        self.f_r1FP = self.momentarm(r1FP)

        pickle_off = open("r2ED.pickle","rb")
        r2ED = pickle.load(pickle_off)
        self.f_r2ED = self.momentarm(r2ED)

        pickle_off = open("r2FP.pickle","rb")
        r2FP = pickle.load(pickle_off)
        self.f_r2FP = self.momentarm(r2FP)

        pickle_off = open("r3ED.pickle","rb")
        r3ED = pickle.load(pickle_off)
        self.f_r3ED = self.momentarm(r3ED)

        pickle_off = open("r3FP.pickle","rb")
        r3FP = pickle.load(pickle_off)
        self.f_r3FP = self.momentarm(r3FP)


    def tendon_displ(self):
        sol = traject_to_angles(self.x, self.z)
        sol.run()
        self.dLED = self.f_r1ED(sol.MCP_deg) + self.f_r2ED(sol.PIP_deg)
        self.dLFP = self.f_r1FP(sol.MCP_deg) + self.f_r2FP(sol.PIP_deg) + self.f_r3FP(sol.DIP_deg)
        self.dLED = self.dLED.astype(np.float)
        self.dLFP = self.dLFP.astype(np.float)

        plt.figure()
        plt.plot(self.t, sol.MCP_deg, '-', alpha = 1)
        plt.plot(self.t, sol.PIP_deg, '-', alpha = 1)
        plt.plot(self.t, sol.DIP_deg, '-', alpha = 1)
        plt.legend(('MCP','PIP','DIP'), loc=2)
        plt.xlabel('Time [s]')
        plt.ylabel('Joint angle [Deg]')
        plt.xlim((0,1))

        plt.figure()
        plt.plot(self.t, self.dLED*pi*47/4095, 'b,', alpha = 1)
        plt.plot(self.t, self.dLFP*pi*60/4095, 'r,', alpha = 1)
        plt.legend((r'$L_{ED}$',r'$L_{FP}$'), loc=2)
        plt.xlabel('Time [s]')
        plt.ylabel('L [mm]')
        #plt.xlim((0,1))

        plt.show()

        # To smoothen trajectories
        #self.dLED_smooth = savgol_filter(self.dLED, 551, 3)
        #self.dLFP_smooth = savgol_filter(self.dLFP, 551, 3)

        #self.dLED_smooth = self.dLED_smooth[self.lb:self.ub+1]
        #self.dLFP_smooth = self.dLFP_smooth[self.lb:self.ub+1]


    def hyperextension(self):
        self.dLED = self.f_r2ED(self.PIP)
        self.dLFP = self.f_r2FP(self.PIP)
        self.dLED = self.dLED.astype(np.float)
        self.dLFP = self.dLFP.astype(np.float)

    def plots(self):
        plt.figure()
        plt.plot(self.t, self.z, '-', alpha = 1)
        plt.xlabel('t [m]')
        plt.ylabel('z [m]')

        plt.figure()
        plt.plot(self.x, self.z, '-', alpha = 1)
        plt.xlabel('x [m]')
        plt.xlim((0.30,0.0))
        plt.ylabel('z [m]')
        plt.ylim((0.3,0.0))

        plt.figure()
        plt.plot(self.t, self.dLED*pi*47/4095, 'b,', alpha = 1)
        #plt.plot(self.t[self.lb:self.ub+1]-1, self.dLED_smooth*pi*47/4095, 'b-', alpha = 1)
        plt.plot(self.t, self.dLFP*pi*60/4095, 'r,', alpha = 1)
        #plt.plot(self.t[self.lb:self.ub+1]-1, self.dLFP_smooth*pi*60/4095, 'r-', alpha = 1)
        #plt.legend((r'$L_{ED}$ measured', r'$L_{ED}$ smoothened', r'$L_{FP}$ measured', r'$L_{FP}$ smoothened'), loc=2)
        plt.xlabel('Time [s]')
        plt.ylabel('L [mm]')
        plt.xlim((0,1))
        plt.savefig('smoothened_displacements.eps',format='eps')

        plt.figure()
        plt.plot(self.dLED[self.lb:self.ub+1]*pi*47/4095, self.dLFP[self.lb:self.ub+1]*pi*60/4095, 'b-,', alpha = 1)
        #plt.plot(self.dLED_smooth*pi*47/4095, self.dLFP_smooth*pi*60/4095, 'r-', alpha = 1)
        #plt.legend((r'calculated', r'smoothened'), loc=2)
        plt.xlabel('$L_{ED}$ [mm]')
        plt.ylabel('$L_{FP}$ [mm]')
        plt.savefig('smoothened_trajectory.eps',format='eps')

        plt.show()

if __name__ == '__main__':
    params = {'legend.fontsize': 'x-large',
          'figure.figsize': (10, 10),
         'axes.labelsize': 32,
         'axes.titlesize': 32,
         'xtick.labelsize': 20,
         'ytick.labelsize': 20}
    pylab.rcParams.update(params)

    MCP = [[0 for i in np.arange(0, 180)], [0 for i in np.arange(0, 180)]]
    MCP = np.array(sum(MCP, []))
    PIP = [[i*(-30/180) for i in np.arange(0, 180)], [(180 - i)*(-30/180) for i in np.arange(0, 180)]]
    PIP = np.array(sum(PIP, []))
    t = np.array([i/MCP.size for i in np.arange(0, MCP.size)])

    ik = inverse_kinematics(MCP, PIP, t)
    ik.load_data()
    ik.hyperextension()
    LED_he = ik.dLED/4*6
    LFP_he = ik.dLFP/12*20

    pickle_in = open("x_traject_%s.pickle"%sys.argv[1],"rb")
    x_vect = pickle.load(pickle_in)

    pickle_in = open("z_traject_%s.pickle"%sys.argv[1],"rb")
    z_vect = pickle.load(pickle_in)

    pickle_in = open("time_traject_%s.pickle"%sys.argv[1],"rb")
    t = np.array(pickle.load(pickle_in))
    t_start = t[0]
    t_end = t[1]
    delta_t = t_end - t_start

    ik2 = inverse_kinematics(np.array(x_vect), np.array(z_vect), t)
    ik2.load_data()
    ik2.tendon_displ()
    #ik.plots()

    LED = np.concatenate((np.array(LED_he), np.array(ik2.dLED)))
    LFP = np.concatenate((np.array(LFP_he), np.array(ik2.dLFP)))

    t = np.array([i/LED.size for i in np.arange(0, len(LED))])

    pickle_out = open("dLED_hyper_%s.pickle"%sys.argv[1],"wb")
    pickle.dump(LED, pickle_out)

    pickle_out = open("dLFP_hyper_%s.pickle"%sys.argv[1],"wb")
    pickle.dump(LFP, pickle_out)
    
    pickle_out = open("time_traject_hyper_%s.pickle"%sys.argv[1],"wb")
    pickle.dump(t, pickle_out)

    # pickle_out = open("dLED_%s.pickle"%sys.argv[1],"wb")
    # pickle.dump(ik2.dLED, pickle_out)
    #
    # pickle_out = open("dLFP_%s.pickle"%sys.argv[1],"wb")
    # pickle.dump(ik2.dLFP, pickle_out)
    #
    # pickle_out = open("time_traject_%s.pickle"%sys.argv[1],"wb")
    # pickle.dump(t, pickle_out)

    plt.figure()
    plt.plot(t, LED*pi*47/4095, 'b,', alpha = 1)
    plt.plot(t, LFP*pi*60/4095, 'r,', alpha = 1)
    plt.xlabel('Time [s]')
    plt.ylabel('L [mm]')
    plt.xlim((0,1))

    plt.show()

    #pickle_out = open("dLED_keystroke.pickle","wb")
    #pickle.dump(ik.dLED_smooth, pickle_out)

    #pickle_out = open("dLFP_keystroke.pickle","wb")
    #pickle.dump(ik.dLFP_smooth, pickle_out)

    # Variable interval beteen points
    # len = ik.ub - ik.lb
    # x = np.array([0, 0.5, 1])
    # y = np.array([1, 0.5, 1])
    #
    # xp = np.linspace(0, 1, len)
    # p = 0.35*np.cos(2*pi*(xp + 0.22)) + 0.65
    # t = []
    # t_int = []
    # t_int.append(delta_t)
    # t.append(0)
    # for i in range(1, len):
    #     t.append(t[i-1] + delta_t*p[i])
    #     t_int.append(delta_t*p[i])
    # t = np.array(t)
    #
    # plt.figure()
    # plt.plot(xp,t_int)
    # plt.show()

    #t = np.array([i/ik.dLFP_smooth.size for i in range(0,ik.dLFP_smooth.size)])

    #pickle_out = open("time_traject_keystroke.pickle","wb")
    #pickle.dump(t, pickle_out)
