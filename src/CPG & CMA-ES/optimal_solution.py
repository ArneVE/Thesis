# One run with the optimal parameters defined by CMA-ES
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
from math import sin, cos, exp, pi, sqrt, ceil

from Hopf_Oscillator import hopf

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
ADDR_GOAL_VEL           = 108
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
client = natnet.NatClient(client_ip='172.17.220.57', data_port=1511, comm_port=1510)


class optimal():
    def __init__(self, default, initial_values, best, lower_bounds, upper_bounds, X0, x_traj, z_traj):
        self.default = default
        self.initial_values = initial_values
        self.best = best
        self.lower_bounds = lower_bounds
        self.upper_bounds = upper_bounds
        self.X0_init = X0
        self.DP = client.rigid_bodies['DP'] # Assuming a Motive Rigid Body is available that you named "DP"
        self.MP = client.rigid_bodies['MP']
        self.PP = client.rigid_bodies['PP']
        self.MC = client.rigid_bodies['MC']

        self.markersDP = self.DP.markers  # returns a list of markers, each with their own properties
        self.markersMP = self.MP.markers
        self.markersPP = self.PP.markers
        self.markersMC = self.MC.markers

        self.x_opt = []
        self.z_opt = []
        self.data1 = []
        self.data2 = []
        self.ticks = []
        self.x_traj = x_traj
        self.z_traj = z_traj


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
        # Add parameter storage for ED and FP present position value
        addparam_result = groupSyncRead_pos.addParam(self.default['ID_ED'])
        if addparam_result != True:
            #print("[ID:%03d] groupSyncRead addparam failed" % self.default['ID_ED'])
            quit()

        addparam_result = groupSyncRead_pos.addParam(self.default['ID_FP'])
        if addparam_result != True:
            #print("[ID:%03d] groupSyncRead addparam failed" % self.default['ID_FP'])
            quit()

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
        a = self.denormalized[0]
        b = self.denormalized[1]
        w_swing = self.denormalized[2]
        w_stance = self.denormalized[3]
        theta = self.denormalized[4]
        mu = self.denormalized[13]
        k = self.denormalized[14]
        X0 = self.X0_init
        X0[0] = self.denormalized[11]
        X0[2] = self.denormalized[12]
        cpg = hopf(self.denormalized, X0)
        [x1, _, x2, _, CPG_time] = cpg.solve(X0)
        #Hopf_Oscillator.set_param(a, b, w_swing, w_stance, theta, mu, k, X0) # (a, b, mu, w_swing, w_stance, theta, k, X0)
        #[x1, y1, x2, y2, CPG_time] = Hopf_Oscillator.solve(X0)
        self.CPG1 = (x1[::])*(self.denormalized[5])
        self.CPG2 = (x2[::])*(self.denormalized[6])
        self.CPG_time = CPG_time


    # Use the optimal parameters obtained from the CMA-ES algorithm
    def try_solution(self):
        self.error = 0   # variable that is equal to 1 if the servos exceed a certain position
        first_tick = time.time()
        while 1:
            tick = time.time()
            self.t = tick - first_tick
            temp = self.CPG_time - self.t
            index = np.argmin(abs(temp))
            current_ED = ceil(self.CPG1[index])
            current_FP = ceil(self.CPG2[index])
            self.ticks.append(self.t)

            param_current_ED = [DXL_LOBYTE(DXL_LOWORD(current_ED)), DXL_HIBYTE(DXL_LOWORD(current_ED))]
            param_current_FP = [DXL_LOBYTE(DXL_LOWORD(current_FP)), DXL_HIBYTE(DXL_LOWORD(current_FP))]
            groupSyncWrite_cur.addParam(self.default['ID_ED'], param_current_ED)
            groupSyncWrite_cur.addParam(self.default['ID_FP'], param_current_FP)
            groupSyncWrite_cur.txPacket()
            groupSyncWrite_cur.clearParam()

            if (np.sign(current_ED) > 0):
                PWM_ED = ceil(self.denormalized[7])
            else:
                PWM_ED = ceil(self.denormalized[9])

            if (np.sign(current_FP) > 0):
                PWM_FP = ceil(self.denormalized[8])
            else:
                PWM_FP = ceil(self.denormalized[10])

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
            self.x_opt.append(self.DP.markers[self.DP_index1].position[0] - self.x0)
            self.z_opt.append(self.DP.markers[self.DP_index1].position[2] - self.z0)

            if (self.calculate_angle(self.MC, self.PP, self.MC_index1, self.MC_index2, self.PP_index1, self.PP_index2) - self.MCP_angle < -60 or
                self.calculate_angle(self.MC, self.PP, self.MC_index1, self.MC_index2, self.PP_index1, self.PP_index2) - self.MCP_angle  > 80 or
                self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle  < -20 or
                self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle  > 110 or
                pos_ED > self.default['restart_ED'] + 1400 or
                pos_ED < self.default['restart_ED'] - 600 or
                pos_FP > self.default['restart_FP'] + 600 or
                pos_FP < self.default['restart_FP'] - 1800 or
                self.DP.markers[self.DP_index1].position[2] - self.z0 < -0.04 or
                self.DP.markers[self.DP_index1].position[2] - self.z0 > 0.25):

                print("Range exceeded.")

            if (self.t > 10):
                print("Done.")
                packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_GOAL_CUR, 0)
                packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_GOAL_CUR, 0)
                self.error = 0
                break



    def plots(self):
        plt.figure(1)
        plt.plot(self.ticks, self.data1)
        plt.plot(self.ticks, self.data2)
        plt.xlabel("Time [s]")
        plt.ylabel("Position [units]")
        plt.legend(['ED', 'FP'])
        plt.show()

        plt.figure(2)
        plt.plot(self.CPG_time[::], self.CPG1)
        plt.plot(self.CPG_time[::], self.CPG2)
        plt.xlabel("Time [s]")
        plt.ylabel("Current [units]")
        plt.legend(['CPG1', 'CPG2'])
        plt.show()

        plt.figure(3)
        plt.plot(self.x_opt, self.z_opt, '.')
        plt.plot(self.x_traj, self.z_traj, '-')
        plt.ylim(0.00,0.25)
        plt.xlabel("x [m]")
        plt.ylabel("z [m]")
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


    def run(self):
        self.port()
        self.restart()
        while(1):
            if ((int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_ED'], 123)[0])[-1]) == 1) and (int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_FP'], 123)[0])[-1]) == 1)):
                break
        self.setup_OptiTrack()
        time.sleep(10)
        self.setup()
        self.denormalized = self.denormalize(self.best)
        self.create_CPG()
        self.try_solution()
        self.plots()



if __name__ == '__main__':
    model_config = {
        'ID_ED' : 3,
        'ID_FP' : 2,
        'ID_OpenCM' : 200,
        'baudrate' : 1e6,
        'baudrate_val' : 3, # value in the registers that corresponds to a baudrate of 1e6
		'max_current': 200,
        'max_PWM' : 200,
        'restart_ED' : 1030,
        'restart_FP' : 3050
        }


    initial =  [1, 50, 2*pi/1.7, 2*pi/1, 0.7*pi, 150, 150, 600, 600, 600, 600, 0, 0, 1, 1] # initial values of the tunable parameters

    # optimal parameters defined by CMA-ES
    best = [0.13548073672404304, 0.35325991516773203, 0.27983333374094416, 0.9948610659597079, 0.22210453778191153, 0.8563338959285824, 0.7609336926599861, 0.3280657736873627, 0.9878437579740545, 0.35881809237940555, 0.7788594082865437, 0.9777547548093148, 0.12138980866280139, 0, 0]


    #     a      b      w_swing   w_stance      theta   I_ED  I_FP  PWM_ED_POS  PWM_FP_POS  PWM_ED_NEG  PWM_FP_NEG  start_CPG1 start_CPG2  mu  k
    lb = [0.001, 0.001, 2*pi/1.5, 2*pi/1.5,   pi/2,   10,    10,  200,        200,        200,        200,        -1,        -1,         1,  1]   # lower bounds of the tunable parameters
    ub = [30,    30,    2*pi/0.8, 2*pi/0.8,   4*pi/2, 150,   150, 600,        600,        600,        600,         1,         1,         1,  1]   # upper bounds of the tunable parameters
    X0_init = [0.0, -1.0, 0.0, 1.0]

    pickle_off = open("x_traject.pickle","rb")
    x_traj = pickle.load(pickle_off)
    pickle_off = open("z_traject.pickle","rb")
    z_traj = pickle.load(pickle_off)
    pickle_off = open("time_traject.pickle","rb")
    time_traj = pickle.load(pickle_off)

    exp = optimal(model_config, initial, best, lb, ub, X0_init, x_traj, z_traj)
    exp.run()
