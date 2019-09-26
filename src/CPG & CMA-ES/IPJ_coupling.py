# Optimization of the CPGs with CMA-ES
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

import Taga_Oscillator
import VDP_Oscillator
import Hopf_Oscillator

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_BAUD_RATE          = 8
ADDR_OPERATING_MODE     = 11
ADDR_TORQUE_ENABLE      = 64
ADDR_CUR_LIMIT          = 38
ADDR_GOAL_PWM           = 100
ADDR_GOAL_CUR           = 102
ADDR_GOAL_VEL           = 104
ADDR_GOAL_POSITION      = 116
ADDR_TICK               = 120
ADDR_MOV                = 122
ADDR_PRES_PWM           = 124
ADDR_PRES_CUR           = 126
ADDR_PRES_VEL           = 128
ADDR_PRES_POSITION      = 132

# Data Byte Length
LEN_1                   = 1
LEN_2                   = 2
LEN_4                   = 4

# Protocol version
PROTOCOL_VERSION        = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DEVICENAME              = 'COM3'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE           = 1                 # Value for enabling the torque
TORQUE_DISABLE          = 0                 # Value for disabling the torque
TORQUE_CONTROL_MODE     = 0                 # Value for torque control mode (operating mode)
VELOCITY_CONTROL_MODE   = 1                 # Value for velocity control mode (operating mode)
POSITION_CONTROL_MODE   = 3

ticks = []
data1 = []
data2 = []

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite and GroupSyncRead instances
groupSyncWrite_cur = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CUR, LEN_2)
groupSyncWrite_PWM = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_PWM, LEN_2)
groupSyncRead_pos = GroupSyncRead(portHandler, packetHandler, ADDR_PRES_POSITION, LEN_4)


# client_ip changes each time a new Motive project is set up
client = natnet.NatClient(client_ip='10.10.130.130', data_port=1511, comm_port=1510)


class coupling():
    def __init__(self, default):
        self.default = default

        self.DP = client.rigid_bodies['DP'] # Assuming a Motive Rigid Body is available that you named "DP"
        self.MP = client.rigid_bodies['MP']
        self.PP = client.rigid_bodies['PP']
        self.MC = client.rigid_bodies['MC']

        self.markersDP = self.DP.markers  # returns a list of markers, each with their own properties
        self.markersMP = self.MP.markers
        self.markersPP = self.PP.markers
        self.markersMC = self.MC.markers

        self.PIP_traj = []
        self.DIP_traj = []


    # Set all important parameters
    def port(self):
        # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if portHandler.setBaudRate(self.default['baudrate']):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()


    def setup(self):
        ################# SET BAUDRATE OPENCM9.04 #################
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_OpenCM'], 12, self.default['baudrate_val'])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Baudrate OpenCM9.04 changed.")


        ################# TORQUE DISABLE #################
        # Disable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and torque is disabled" % self.default['ID_FP'])

        # Disable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and torque is disabled" % self.default['ID_ED'])


    def restart(self):
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_GOAL_PWM, self.default['max_PWM'])   # Set PWM (max velocity) of the ED
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_GOAL_PWM, self.default['max_PWM'])   # Set PWM (max velocity) of the FP
        packetHandler.write4ByteTxRx(portHandler, self.default['ID_ED'], ADDR_GOAL_POSITION, self.default['restart_ED'])
        packetHandler.write4ByteTxRx(portHandler, self.default['ID_FP'], ADDR_GOAL_POSITION, self.default['restart_FP'])


    ## In the first experiment the ED will be subjected to a certain current curve and the FP will remain in its position
    ## In a second experiment the FP wil be subjected to a curve in counter phase
    def try_solution(self):
        start_time = time.time()
        temp_time = start_time
        self.PIP_traj.append(0)
        self.DIP_traj.append(0)
        while 1:
            if (time.time() - temp_time > 1/150 and
                abs(self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle - self.PIP_traj[-1]) < 5 and
                abs(self.calculate_angle(self.MP, self.DP, self.MP_index1, self.MP_index2, self.DP_index1, self.DP_index2) - self.DIP_angle - self.DIP_traj[-1]) < 5):

                self.PIP_traj.append(self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle)
                self.DIP_traj.append(self.calculate_angle(self.MP, self.DP, self.MP_index1, self.MP_index2, self.DP_index1, self.DP_index2) - self.DIP_angle)
                temp_time = time.time()

            # Measure for x seconds (here 60)
            if (time.time() - start_time > 60):
                print("Measurement finished.")
                break


    def plots(self):
        pickle_on = open("PIP.pickle","wb")
        pickle.dump(self.PIP_traj, pickle_on)
        pickle_on = open("DIP.pickle","wb")
        pickle.dump(self.DIP_traj, pickle_on)

        plots = zip(self.PIP_traj, self.DIP_traj)
        with open('IPJ_coupling.csv', "w") as f:
            writer = csv.writer(f)
            writer.writerow(['PIP,DIP'])
            for row in plots:
                writer.writerow(row)

        plt.figure(1)
        plt.plot(self.PIP_traj, self.DIP_traj, '-')
        plt.xlabel(r'$\theta_{PIP}$')
        plt.ylabel(r'$\theta_{DIP}$')

        plt.show()


    # Get positions of 2 markers that will define the angle
    # Choose markers that are located on (approximately) a straight line
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


    def run(self):
        self.port()

        self.restart()

        while(1):
            if ((int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_ED'], 123)[0])[-1]) == 1) and (int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_FP'], 123)[0])[-1]) == 1)):
                break
        time.sleep(0.5)

        self.setup_OptiTrack()

        time.sleep(2)
        print("Start measurement.")
        self.setup()

        self.try_solution()

        self.plots()


if __name__ == '__main__':
    model_config = {
        'ID_ED' : 3,
        'ID_FP' : 2,
        'ID_OpenCM' : 200,
        'baudrate' : 1e6,
        'baudrate_val' : 3,
		'max_current': 200,
        'max_PWM' : 200,
        'restart_ED' : 1030,
        'restart_FP' : 3050
        }

    exp = coupling(model_config)
    exp.run()
