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
ADDR_PWM_LIMIT          = 36
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


class CMAES():
    def __init__(self, default, initial_values, lower_bounds, upper_bounds, X0, x_traj, z_traj, time_traj):
        self.default = default
        self.initial_values = initial_values
        self.lower_bounds = lower_bounds
        self.upper_bounds = upper_bounds
        self.X0_init = X0
        self.params = []

        self.DP = client.rigid_bodies['DP'] # Assuming a Motive Rigid Body is available that you named "DP"
        self.MP = client.rigid_bodies['MP']
        self.PP = client.rigid_bodies['PP']
        self.MC = client.rigid_bodies['MC']

        self.markersDP = self.DP.markers  # returns a list of markers, each with their own properties
        self.markersMP = self.MP.markers
        self.markersPP = self.PP.markers
        self.markersMC = self.MC.markers

        self.x_traj = x_traj
        self.z_traj = z_traj
        self.time_traj = time_traj
        self.data1 = []
        self.data2 = []

        self.reward = 0
        self.count = 0


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


        ################# SET OPERATING MODE #################
        # Torque control mode Dynamixel#1
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_OPERATING_MODE, TORQUE_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and set in torque control mode" % self.default['ID_FP'])

        # Torque control mode Dynamixel#2
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_OPERATING_MODE, TORQUE_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and set in torque control mode" % self.default['ID_ED'])


        ################# SET CURRENT LIMIT #################
        # Torque control mode Dynamixel#1
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_CUR_LIMIT, self.default['max_current'])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and current limit adjusted" % self.default['ID_FP'])

        # Torque control mode Dynamixel#2
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_CUR_LIMIT, self.default['max_current'])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and current limit adjusted" % self.default['ID_ED'])

        ################# SET PWM LIMIT #################
        # Torque control mode Dynamixel#1
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_PWM_LIMIT, 800)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and current limit adjusted" % self.default['ID_FP'])

        # Torque control mode Dynamixel#2
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_PWM_LIMIT, 800)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and current limit adjusted" % self.default['ID_ED'])


        ################# ENABLE TORQUE #################
        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and torque is enabled" % self.default['ID_FP'])

        # Enable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and torque is enabled" % self.default['ID_ED'])


        ################# STORAGE POSITION #################
        if self.count == 0:
            # Add parameter storage for ED and FP present position value
            addparam_result = groupSyncRead_pos.addParam(self.default['ID_ED'])
            if addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % self.default['ID_ED'])
                quit()

            addparam_result = groupSyncRead_pos.addParam(self.default['ID_FP'])
            if addparam_result != True:
                print("[ID:%03d] groupSyncRead addparam failed" % self.default['ID_FP'])
                quit()

            self.count += 1


        ################# EXTRA SETTINGS #################
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], 76, 0)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], 76, 0)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], 78, 0)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], 78, 0)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], 84, 0)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], 84, 0)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_GOAL_PWM, self.default['max_PWM'])   # Set PWM (max velocity) of the ED
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_GOAL_PWM, self.default['max_PWM'])   # Set PWM (max velocity) of the FP


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


    # Create the CPGs
    # Good starting parameters:
    # a = 1
    # b = 50
    # mu = 1
    # w_swing = 2*pi/1.7
    # w_stance = 2*pi/1
    # theta = 0.3*pi
    # k = 1
    # X0 = [0.0, -1.0, 0.0, 1.0]
    def create_CPG(self):
        a = self.params[0]
        b = self.params[1]
        w_swing = self.params[2]
        w_stance = self.params[3]
        theta = self.params[4]
        mu = self.params[13]
        k = self.params[14]
        X0 = self.X0_init
        X0[0] = self.params[11]
        X0[2] = self.params[12]
        Hopf_Oscillator.set_param(a, b, w_swing, w_stance, theta, mu, k, X0) # (a, b, mu, w_swing, w_stance, theta, k, X0)
        [x1, y1, x2, y2, CPG_time] = Hopf_Oscillator.solve(X0)
        self.CPG1 = (x1[::])*(self.params[5])
        self.CPG2 = (x2[::])*(self.params[6])
        self.CPG_time = CPG_time


    ## Try a solution proposed by the CMA-ES algorithm
    def try_solution(self):
        self.error = 0   # variable that is equal to 1 if the servos exceed a certain position
        self.x_cmaes = []
        self.z_cmaes = []
        self.time_cmaes = []
        first_tick = time.time()
        temp_time = first_tick
        while 1:
            tick = time.time()
            self.t = tick - first_tick
            temp = self.CPG_time - self.t
            index = np.argmin(abs(temp))
            current_ED = ceil(self.CPG1[index])
            current_FP = ceil(self.CPG2[index])
            self.time_cmaes.append(self.t)

            param_current_ED = [DXL_LOBYTE(DXL_LOWORD(current_ED)), DXL_HIBYTE(DXL_LOWORD(current_ED))]
            param_current_FP = [DXL_LOBYTE(DXL_LOWORD(current_FP)), DXL_HIBYTE(DXL_LOWORD(current_FP))]
            groupSyncWrite_cur.addParam(self.default['ID_ED'], param_current_ED)
            groupSyncWrite_cur.addParam(self.default['ID_FP'], param_current_FP)
            groupSyncWrite_cur.txPacket()
            groupSyncWrite_cur.clearParam()

            if (np.sign(current_ED) > 0):
                PWM_ED = ceil(self.params[7])
            else:
                PWM_ED = ceil(self.params[9])

            if (np.sign(current_FP) > 0):
                PWM_FP = ceil(self.params[8])
            else:
                PWM_FP = ceil(self.params[10])

            param_PWM_ED = [DXL_LOBYTE(DXL_LOWORD(PWM_ED)), DXL_HIBYTE(DXL_LOWORD(PWM_ED))]
            param_PWM_FP = [DXL_LOBYTE(DXL_LOWORD(PWM_FP)), DXL_HIBYTE(DXL_LOWORD(PWM_FP))]
            groupSyncWrite_PWM.addParam(self.default['ID_ED'], param_PWM_ED)
            groupSyncWrite_PWM.addParam(self.default['ID_FP'], param_PWM_FP)
            groupSyncWrite_PWM.txPacket()
            groupSyncWrite_PWM.clearParam()

            groupSyncRead_pos.txRxPacket()
            pos_ED = groupSyncRead_pos.getData(self.default['ID_ED'], ADDR_PRES_POSITION, LEN_4)
            pos_FP = groupSyncRead_pos.getData(self.default['ID_FP'], ADDR_PRES_POSITION, LEN_4)
            self.data1.append(pos_ED)
            self.data2.append(pos_FP)
            x = self.DP.markers[self.DP_index1].position[0] - self.x0
            z = self.DP.markers[self.DP_index1].position[2] - self.z0
            self.x_cmaes.append(x)
            self.z_cmaes.append(z)

            # If servos exceed a certain range or the finger exceeds a certain angle -> quit
            if (self.calculate_angle(self.MC, self.PP, self.MC_index1, self.MC_index2, self.PP_index1, self.PP_index2) - self.MCP_angle < -60 or
                self.calculate_angle(self.MC, self.PP, self.MC_index1, self.MC_index2, self.PP_index1, self.PP_index2) - self.MCP_angle  > 85 or
                self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle  < -30 or
                self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle  > 110 or
                self.calculate_angle(self.MP, self.DP, self.MP_index1, self.MP_index2, self.DP_index1, self.DP_index2) - self.DIP_angle  < -20 or
                self.calculate_angle(self.MP, self.DP, self.MP_index1, self.MP_index2, self.DP_index1, self.DP_index2) - self.DIP_angle  > 100 or
                pos_ED > 2400 or
                pos_ED < 400 or
                pos_FP > 3600 or
                pos_FP < 1200 or
                self.DP.markers[self.DP_index1].position[2] - self.z0 < -0.04 or
                self.DP.markers[self.DP_index1].position[2] - self.z0 > 0.25):

                print("Range exceeded.")

                param_current_ED = [DXL_LOBYTE(DXL_LOWORD(0)), DXL_HIBYTE(DXL_LOWORD(0))]
                param_current_FP = [DXL_LOBYTE(DXL_LOWORD(0)), DXL_HIBYTE(DXL_LOWORD(0))]
                groupSyncWrite_cur.addParam(self.default['ID_ED'], param_current_ED)
                groupSyncWrite_cur.addParam(self.default['ID_FP'], param_current_FP)
                groupSyncWrite_cur.txPacket()
                groupSyncWrite_cur.clearParam()
                self.error = 1
                break

            # Stop after 10 seconds
            if (self.t > 10):
                print("Successful run.")

                param_current_ED = [DXL_LOBYTE(DXL_LOWORD(0)), DXL_HIBYTE(DXL_LOWORD(0))]
                param_current_FP = [DXL_LOBYTE(DXL_LOWORD(0)), DXL_HIBYTE(DXL_LOWORD(0))]
                groupSyncWrite_cur.addParam(self.default['ID_ED'], param_current_ED)
                groupSyncWrite_cur.addParam(self.default['ID_FP'], param_current_FP)
                groupSyncWrite_cur.txPacket()
                groupSyncWrite_cur.clearParam()
                self.error = 0
                break


    def plots(self):
        # plt.figure(1)
        # plt.plot(self.time_cmaes, self.data1)
        # plt.plot(self.time_cmaes, self.data2)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Position [units]")
        # plt.legend(['ED', 'FP'])
        # plt.show()

        # plt.figure(2)
        # plt.plot(self.CPG_time[::], self.CPG1)
        # plt.plot(self.CPG_time[::], self.CPG2)
        # plt.xlabel("Time [s]")
        # plt.ylabel("Current [units]")
        # plt.legend(['CPG1', 'CPG2'])
        # plt.show()

        plt.figure(3)
        plt.plot(self.x_cmaes, self.z_cmaes, '.')
        plt.plot(self.x_traj, self.z_traj, '-')
        plt.ylim(0.00,0.25)
        plt.xlabel("x")
        plt.ylabel("z")
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


    def normalize(self):
        normalized_initial = np.array(self.initial_values)/(np.array(self.upper_bounds) - np.array(self.lower_bounds))

        return normalized_initial


    def denormalize(self, normalized):
        denormalized = np.multiply((np.array(self.upper_bounds) - np.array(self.lower_bounds)), normalized) + np.array(self.lower_bounds)

        return denormalized


    # CMA-ES algorithm
    def run(self):
        self.port()
        best = cma.optimization_tools.BestSolution()

        N = len(self.initial_values) - 2
        pop_size = 4 + np.floor(3*log(N))

        es = cma.CMAEvolutionStrategy(N * [0.5], 0.5, {'bounds': [0, 1], 'popsize': pop_size, 'maxiter': 250, 'verb_append': best.evalsall})
        logger = cma.CMADataLogger().register(es, append = best.evalsall)
        while not es.stop():
            self.restart()

            while(1):
                if ((int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_ED'], 123)[0])[-1]) == 1) and (int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_FP'], 123)[0])[-1]) == 1)):
                    break
            time.sleep(0.5)

            self.setup_OptiTrack()

            solutions = es.ask()
            reward_vect = []
            for i in range(len(solutions)):
                norm = np.concatenate([solutions[i], [0, 0]])
                self.params = self.denormalize(norm)
                self.reward = 0

                self.setup()

                self.create_CPG()

                self.try_solution()

                index = np.argmin(abs(np.array(self.time_traj) - self.time_cmaes[0]))
                self.reward += sqrt((self.x_traj[index] - self.x_cmaes[0])**2 + (self.z_traj[index] - self.z_cmaes[0])**2)

                # if a marker jumps to a random position
                for i in range(len(self.x_cmaes) - 1):
                    if (abs(self.x_cmaes[i + 1] - self.x_cmaes[i]) > 0.01):
                        self.x_cmaes[i + 1] = self.x_cmaes[i]
                        self.z_cmaes[i + 1] = self.z_cmaes[i]
                    elif(abs(self.z_cmaes[i + 1] - self.z_cmaes[i]) > 0.01):
                        self.x_cmaes[i + 1] = self.x_cmaes[i]
                        self.z_cmaes[i + 1] = self.z_cmaes[i]

                    index = np.argmin(abs(np.array(self.time_traj) - self.time_cmaes[i + 1]))
                    self.reward += sqrt((self.x_traj[index] - self.x_cmaes[i + 1])**2 + (self.z_traj[index] - self.z_cmaes[i + 1])**2)

                if self.t < 1:
                    self.reward /= math.exp(self.t - 1)**5
                else:
                    self.reward /= (self.t)**(1.5)


                print("Total: ", self.reward)
                reward_vect.append(self.reward)

                self.restart()

                while(1):
                    if ((int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_ED'], 123)[0])[-1]) == 1) and (int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_FP'], 123)[0])[-1]) == 1)):
                        break
                time.sleep(0.5)

            es.tell(solutions, reward_vect)
            logger.add()
            es.disp()
        #self.plots()
        best.update(es.best)



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

    initial =  [1, 50, 2*pi/1.7, 2*pi/1, 0.7*pi, 150, 150, 600, 600, 600, 600, 0, 0, 1, 1] # initial values of the tunable parameters

    #     a      b      w_swing   w_stance      theta   I_ED  I_FP  PWM_ED_POS  PWM_FP_POS  PWM_ED_NEG  PWM_FP_NEG  start_CPG1 start_CPG2  mu  k
    lb = [0.001, 0.001, 2*pi/1.5,   2*pi/1.5,   pi/2,   10,    10,  200,        200,        200,        200,        -1,        -1,         1,  1]   # lower bounds of the tunable parameters
    ub = [30,    30,    2*pi/0.8, 2*pi/0.8,     4*pi/2, 150,   150, 600,        600,        600,        600,         1,         1,         1,  1]   # upper bounds of the tunable parameters
    X0_init = [0.0, -1.0, 0.0, 1.0]

    # Desired polynomial
    pickle_off = open("x_traject.pickle","rb")
    x_traj = pickle.load(pickle_off)
    pickle_off = open("z_traject.pickle","rb")
    z_traj = pickle.load(pickle_off)
    pickle_off = open("time_traject.pickle","rb")
    time_traj = pickle.load(pickle_off)

    exp = CMAES(model_config, initial, lb, ub, X0_init, x_traj, z_traj, time_traj)
    exp.run()
