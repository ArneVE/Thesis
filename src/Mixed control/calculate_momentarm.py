# Optimization of the CPGs with CMA-ES
"""
    Run as:
    python .\calculate_momentarm.py 'joint' (MCP, PIP, DIP) 'outputfileED.pickle' 'outputfileFP.pickle'
    Example: python .\calculate_momentarm.py 'MCP' 'r1ED.pickle' 'r1FP.pickle'
"""

import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import sys
import csv
import xlsxwriter
import time
import pandas as pd
import numpy as np
import pickle
import cma
import math
from math import sin, cos, exp, pi, sqrt, ceil, inf, log

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
ADDR_PWM_LIMIT          = 36
ADDR_CUR_LIMIT          = 38
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_PWM           = 100
ADDR_GOAL_CUR           = 102
ADDR_GOAL_VEL           = 104
ADDR_GOAL_POSITION      = 116
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
PWM_CONTROL_MODE        = 16


# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite and GroupSyncRead instances
groupSyncWrite_vel = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VEL, LEN_4)
groupSyncWrite_PWM = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_PWM, LEN_2)
groupSyncRead_pos = GroupSyncRead(portHandler, packetHandler, ADDR_PRES_POSITION, LEN_4)


# client_ip changes each time a new Motive project is set up
client = natnet.NatClient(client_ip='10.10.130.130',
                          data_port=1511,
                          comm_port=1510)


class Momentarm():
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

        self.L_ED = []
        self.L_FP = []
        self.theta = []

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
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and set in torque control mode" % self.default['ID_FP'])

        # Torque control mode Dynamixel#2
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_OPERATING_MODE, PWM_CONTROL_MODE)
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
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_PWM_LIMIT, self.default['max_PWM'])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Dynamixel#%d has been successfully connected and current limit adjusted" % self.default['ID_FP'])

        # Torque control mode Dynamixel#2
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_PWM_LIMIT, self.default['max_PWM'])
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


        ################# DEFINE INITIAL POSITION #################
        groupSyncRead_pos.txRxPacket()
        self.init_ED = groupSyncRead_pos.getData(self.default['ID_ED'], ADDR_PRES_POSITION, LEN_4)
        self.init_FP = groupSyncRead_pos.getData(self.default['ID_FP'], ADDR_PRES_POSITION, LEN_4)


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


    def plots(self, joint):
        plt.figure(1)
        plt.plot(self.theta, self.L_ED, '-')
        plt.xlabel(r'$\theta_{%s}$' % joint)
        plt.ylabel(r'$L_{ED}$')

        plt.figure(2)
        plt.plot(self.theta, self.L_FP, '-')
        plt.xlabel(r'$\theta_{%s}$' % joint)
        plt.ylabel(r'$L_{FP}$')

        plt.show()


    def measure(self, joint):
        # set the parameters for the calculation of the joint angle
        if joint == 'MCP':
            P1 = self.MC
            P2 = self.PP
            P1_index1 = self.MC_index1
            P1_index2 = self.MC_index2
            P2_index1 = self.PP_index1
            P2_index2 = self.PP_index2
            init_angle = self.MCP_angle

        elif joint == 'PIP':
            P1 = self.PP
            P2 = self.MP
            P1_index1 = self.PP_index1
            P1_index2 = self.PP_index2
            P2_index1 = self.MP_index1
            P2_index2 = self.MP_index2
            init_angle = self.PIP_angle

        elif joint == 'DIP':
            P1 = self.MP
            P2 = self.DP
            P1_index1 = self.MP_index1
            P1_index2 = self.MP_index2
            P2_index1 = self.DP_index1
            P2_index2 = self.DP_index2
            init_angle = self.DIP_angle

        else:
            print("INVALID JOINT")
            sys.exit()

        # Initial PWM servos
        param_PWM_ED = [DXL_LOBYTE(DXL_LOWORD(self.default['PWM_ED'])), DXL_HIBYTE(DXL_LOWORD(self.default['PWM_ED']))]
        param_PWM_FP = [DXL_LOBYTE(DXL_LOWORD(-self.default['PWM_FP'])), DXL_HIBYTE(DXL_LOWORD(-self.default['PWM_FP']))]
        groupSyncWrite_PWM.addParam(self.default['ID_ED'], param_PWM_ED)
        groupSyncWrite_PWM.addParam(self.default['ID_FP'], param_PWM_FP)
        groupSyncWrite_PWM.txPacket()
        groupSyncWrite_PWM.clearParam()

        start_time = time.time()

        while time.time() - start_time < 20:
            # Change PWM if a certain angle is reached
            if (self.calculate_angle(P1, P2, P1_index1, P1_index2, P2_index1, P2_index2) - init_angle > 103):
                param_PWM_ED = [DXL_LOBYTE(DXL_LOWORD(-self.default['PWM_ED'])), DXL_HIBYTE(DXL_LOWORD(-self.default['PWM_ED']))]
                param_PWM_FP = [DXL_LOBYTE(DXL_LOWORD(self.default['PWM_FP'] - 200)), DXL_HIBYTE(DXL_LOWORD(self.default['PWM_FP'] - 200))]
                groupSyncWrite_PWM.addParam(self.default['ID_ED'], param_PWM_ED)
                groupSyncWrite_PWM.addParam(self.default['ID_FP'], param_PWM_FP)
                groupSyncWrite_PWM.txPacket()
                groupSyncWrite_PWM.clearParam()

            elif (self.calculate_angle(P1, P2, P1_index1, P1_index2, P2_index1, P2_index2) - init_angle < -40):
                param_PWM_ED = [DXL_LOBYTE(DXL_LOWORD(self.default['PWM_ED'])), DXL_HIBYTE(DXL_LOWORD(self.default['PWM_ED']))]
                param_PWM_FP = [DXL_LOBYTE(DXL_LOWORD(-self.default['PWM_FP'])), DXL_HIBYTE(DXL_LOWORD(-self.default['PWM_FP']))]
                groupSyncWrite_PWM.addParam(self.default['ID_ED'], param_PWM_ED)
                groupSyncWrite_PWM.addParam(self.default['ID_FP'], param_PWM_FP)
                groupSyncWrite_PWM.txPacket()
                groupSyncWrite_PWM.clearParam()

            # calculate dL_ED and dL_FP
            groupSyncRead_pos.txRxPacket()
            pos_ED = groupSyncRead_pos.getData(self.default['ID_ED'], ADDR_PRES_POSITION, LEN_4)
            pos_FP = groupSyncRead_pos.getData(self.default['ID_FP'], ADDR_PRES_POSITION, LEN_4)
            self.L_ED.append(pos_ED - self.init_ED)
            self.L_FP.append(pos_FP - self.init_FP)

            # calculate dtheta_ED and dtheta_FP
            self.theta.append(self.calculate_angle(P1, P2, P1_index1, P1_index2, P2_index1, P2_index2) - init_angle)

        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


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


    # CMA-ES algorithm
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

        self.measure(sys.argv[1])

        pickle_on = open(sys.argv[2],"wb")
        pickle.dump([self.L_ED, self.theta], pickle_on)

        pickle_on = open(sys.argv[3],"wb")
        pickle.dump([self.L_FP, self.theta], pickle_on)

        self.plots(sys.argv[1])



if __name__ == '__main__':
    model_config = {
        'ID_ED' : 3,
        'ID_FP' : 2,
        'ID_OpenCM' : 200,
        'baudrate' : 1e6,
        'baudrate_val' : 3,
		'max_current': 100,
        'max_PWM' : 300,
        'restart_ED' : 985,
        'restart_FP' : 3015,
        'PWM_ED' : 120, # Change PWM for different joints (Not constant for all joints)
        'PWM_FP' : 300  # Change PWM for different joints (Not constant for all joints)
        }


    exp = Momentarm(model_config)
    exp.run()
