import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import xlsxwriter
import time
import pandas as pd
import numpy as np
import pickle
import math
from math import sin, cos, exp, pi, sqrt, ceil

# client_ip changes each time a new Motive project is set up
client = natnet.NatClient(client_ip='172.17.221.47', data_port=1511, comm_port=1510)

DP = client.rigid_bodies['DP']
MP = client.rigid_bodies['MP']
PP = client.rigid_bodies['PP']
MC = client.rigid_bodies['MC']


class Read_data():
    def __init__(self):

        self.DP = client.rigid_bodies['DP'] # Assuming a Motive Rigid Body is available that you named "DP"
        self.MP = client.rigid_bodies['MP']
        self.PP = client.rigid_bodies['PP']
        self.MC = client.rigid_bodies['MC']

        self.markersDP = self.DP.markers  # returns a list of markers, each with their own properties
        self.markersMP = self.MP.markers
        self.markersPP = self.PP.markers
        self.markersMC = self.MC.markers

        self.x_traj = []
        self.z_traj = []
        self.time = []


    def setup_OptiTrack(self):
        self.DP_dist = 10
        self.MP_dist = 10
        self.PP_dist = 10
        self.MC_dist = 10
        for i in range(0, 3):
            for j in range(i+1,3):
                if (abs(self.DP.markers[i].position[2] - self.DP.markers[j].position[2]) < self.DP_dist):
                    self.DP_dist, self.DP_index1, self.DP_index2 = self.select_markers(self.DP, i , j)

                if (abs(self.MP.markers[i].position[2] - self.MP.markers[j].position[2]) < self.MP_dist):
                    self.MP_dist, self.MP_index1, self.MP_index2 = self.select_markers(self.MP, i , j)

                if (abs(self.PP.markers[i].position[2] - self.PP.markers[j].position[2]) < self.PP_dist):
                    self.PP_dist, self.PP_index1, self.PP_index2 = self.select_markers(self.PP, i , j)

                if (abs(self.MC.markers[i].position[2] - self.MC.markers[j].position[2]) < self.MC_dist):
                    self.MC_dist, self.MC_index1, self.MC_index2 = self.select_markers(self.MC, i , j)

        # Set the origin at the 3rd marker of the MC
        if (self.MC.markers[0].position[2] < self.MC.markers[1].position[2]):
            if (self.MC.markers[0].position[2] < self.MC.markers[2].position[2]):
                self.MC_index3 = 0
            else:
                self.MC_index3 = 2
        else:
            if (self.MC.markers[1].position[2] < self.MC.markers[2].position[2]):
                self.MC_index3 = 1
            else:
                self.MC_index3 = 2

        self.x0 = self.MC.markers[self.MC_index3].position[0]
        self.z0 = self.MC.markers[self.MC_index3].position[2]

        self.DIP_angle = self.calculate_angle(self.MP, self.DP, self.MP_index1, self.MP_index2, self.DP_index1, self.DP_index2)
        self.PIP_angle = self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2)
        self.MCP_angle = self.calculate_angle(self.MC, self.PP, self.MC_index1, self.MC_index2, self.PP_index1, self.PP_index2)


    # The marker with largest x-value get label 1, the other gets label 2
    def select_markers(self, P, i, j):
        P_dist = abs(P.markers[i].position[2] - P.markers[j].position[2])
        if (P.markers[i].position[0] > P.markers[j].position[0]):
            P_index1 = i
            P_index2 = j
        else:
            P_index1 = j
            P_index2 = i

        return P_dist, P_index1, P_index2


    def calculate_angle(self, P1, P2, i1, j1, i2, j2):
        vP1x = P1.markers[i1].position[0] - P1.markers[j1].position[0]
        vP1z = P1.markers[i1].position[2] - P1.markers[j1].position[2]
        vP2x = P2.markers[i2].position[0] - P2.markers[j2].position[0]
        vP2z = P2.markers[i2].position[2] - P2.markers[j2].position[2]
        angle = math.degrees(math.atan2(vP1x*vP2z - vP1z*vP2x, vP1x*vP2x + vP1z*vP2z))

        return angle


    def record(self):
        start_time = time.time()
        temp_time = start_time
        while time.time() - start_time < 15:

            if time.time() - temp_time > 1/200:
                self.x_traj.append(self.DP.markers[self.DP_index1].position[0] - self.x0)
                self.z_traj.append(self.DP.markers[self.DP_index1].position[2] - self.z0)
                self.time.append(time.time() - start_time)
                temp_time = time.time()

    def write(self):
        pickle_on = open("x_traj.pickle","wb")
        pickle.dump(self.x_traj, pickle_on)
        pickle_on = open("z_traj.pickle","wb")
        pickle.dump(self.z_traj, pickle_on)
        pickle_on = open("time.pickle","wb")
        pickle.dump(self.time, pickle_on)

        plt.figure(1)
        plt.plot(self.x_traj, self.z_traj, '.')
        plt.ylim(0,0.25)
        plt.xlabel("x")
        plt.ylabel("z")
        plt.show()


time.sleep(3)
print("Start")

rd = Read_data()

rd.setup_OptiTrack()

print("Start")
rd.record()

print("Stop")

rd.write()
