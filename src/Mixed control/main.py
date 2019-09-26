# ED and FP in position control and IO is force controlled
###################### USAGE ######################
"""
python .\main.py flag traject_type

Arguments: flag: 0 or 1
                 -> 0 if the tendon displacements must still be calculated from the trajectory
                 -> 1 if the tendon displacements are already calculated and read from a .pkl file
           traject_type: eg. 0MCP, 0PIP

"""
import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import sys
import csv
import xlsxwriter
import time
import serial
import pandas as pd
import numpy as np
import pickle
import cma
import math
from math import sin, cos, exp, pi, sqrt, ceil, inf, log
from scipy.constants import g
import matplotlib.pylab as pylab

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
from inverse_kinematics import inverse_kinematics


# Control table address
ADDR_BAUD_RATE          = 8
ADDR_OPERATING_MODE     = 11
ADDR_PWM_LIMIT          = 36
ADDR_CUR_LIMIT          = 38
ADDR_TORQUE_ENABLE      = 64
ADDR_I_VEL              = 76
ADDR_P_VEL              = 78
ADDR_D_POS              = 80
ADDR_I_POS              = 82
ADDR_P_POS              = 84
ADDR_GOAL_PWM           = 100
ADDR_GOAL_CUR           = 102
ADDR_GOAL_VEL           = 104
ADDR_GOAL_POSITION      = 116
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
POSITION_CONTROL_MODE   = 4
PID_CONTROL_MODE        = 5
PWM_CONTROL_MODE        = 16


# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite and GroupSyncRead instances
groupSyncWrite_pos = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_4)
groupSyncWrite_PWM = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_PWM, LEN_2)
groupSyncWrite_cur = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CUR, LEN_2)
groupSyncRead_pos = GroupSyncRead(portHandler, packetHandler, ADDR_PRES_POSITION, LEN_4)
groupSyncRead_vel = GroupSyncRead(portHandler, packetHandler, ADDR_PRES_VEL, LEN_4)



#client_ip changes each time a new Motive project is set up
client = natnet.NatClient(client_ip='10.10.130.133', data_port=1511, comm_port=1510)

ser = serial.Serial('COM6', 9600)


class Experiment():
    def __init__(self, default, x_cal, z_cal, t_cal, flag_traj):
        self.default = default

        self.DP = client.rigid_bodies['DP'] # Assuming a Motive Rigid Body is available that you named "DP"
        self.MP = client.rigid_bodies['MP']
        self.PP = client.rigid_bodies['PP']
        self.MC = client.rigid_bodies['MC']

        self.markersDP = self.DP.markers  # returns a list of markers, each with their own properties
        self.markersMP = self.MP.markers
        self.markersPP = self.PP.markers
        self.markersMC = self.MC.markers

        # Joint angles
        self.PIP = []
        self.DIP = []
        self.MCP = []
        self.t_angles = []

        # Reference trajectory
        self.x_cal = x_cal
        self.z_cal = z_cal
        self.t_cal = t_cal

        # Measured trajectory
        self.x_meas = []
        self.z_meas = []

        self.flag_traj = flag_traj

        # Measured servo position
        self.LED_meas = []
        self.LFP_meas = []
        self.t_meas = []

        # Calibration parameters & force vectors
        ######################################
        """
        5 = IO
        2 = FP
        3 = ED
        """
        ######################################
        self.n_cell = [4, 5, 2, 3]
        self.V0 = [212.41940255774003, 289.4916253101737, 326.32871416392908, 223.38865312652007]
        self.Vmax = [861.0096605742118, 909.6029776674937, 1042.7371453747471, 911.1855386923655] # [+649, +620, +716, +688]
        self.wmax = 4.539
        self.wIO = []
        self.wFP = []
        self.wED = []
        self.t_w = []

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
        # Disable FP Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # Disable ED Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        # Disable IO Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


        ################# SET OPERATING MODE #################
        # Position control mode FP
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)

        # Position control mode ED
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)

        # Torque based position control mode IO
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_OPERATING_MODE, PWM_CONTROL_MODE)


        ################# SET CURRENT LIMIT #################
        # Current limit FP
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_CUR_LIMIT, 200)

        # Current limit ED
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_CUR_LIMIT, 200)

        # Current limit IO
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_IO'], ADDR_CUR_LIMIT, self.default['max_current'])


        ################# SET PWM LIMIT #################
        # PWM limit FP
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_PWM_LIMIT, self.default['max_PWM'])

        # PWM limit ED
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_PWM_LIMIT, self.default['max_PWM'])

        # PWM limit IO
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_IO'], ADDR_PWM_LIMIT, self.default['max_PWM'])


        ################# ENABLE TORQUE #################
        # Enable FP Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        # Enable ED Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        # Enable IO Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


        ################# SET PID PARAMETERS IO #################
        # P parameter
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_P_POS, 600)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_P_POS, 600)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_IO'], ADDR_P_POS, 30)

        # I parameter
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_I_POS, 0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_I_POS, 0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_IO'], ADDR_I_POS, 0)

        # D parameter
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_D_POS, 0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_D_POS, 0)
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, self.default['ID_IO'], ADDR_D_POS, 100)


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

            addparam_result = groupSyncRead_vel.addParam(self.default['ID_FP'])
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
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_OPERATING_MODE, POSITION_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_ED'], ADDR_GOAL_PWM, 300)   # Set PWM (max velocity) of the ED
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_FP'], ADDR_GOAL_PWM, 300)   # Set PWM (max velocity) of the FP
        packetHandler.write2ByteTxRx(portHandler, self.default['ID_IO'], ADDR_GOAL_PWM, 100)   # Set PWM (max velocity) of the IO
        packetHandler.write4ByteTxRx(portHandler, self.default['ID_ED'], ADDR_GOAL_POSITION, self.default['restart_ED'])
        packetHandler.write4ByteTxRx(portHandler, self.default['ID_FP'], ADDR_GOAL_POSITION, self.default['restart_FP'])
        packetHandler.write4ByteTxRx(portHandler, self.default['ID_IO'], ADDR_GOAL_POSITION, self.default['restart_IO'])


    def plots(self):
        plt.figure(1)
        plt.plot(self.x_cal, self.z_cal, '-')
        plt.plot(self.x_meas, self.z_meas, '-')
        plt.legend(('p(t) desired', 'p(t) measured'))
        plt.xlim((0.3,0.08))
        plt.ylim((0.22,0))
        plt.xlabel('x [m]')
        plt.ylabel('z [m]')

        plt.figure(2)
        plt.plot(self.LED_cal*pi*47/4095, self.LFP_cal*pi*60/4095, '-')
        plt.plot(self.LED_meas*pi*47/4095, self.LFP_meas*pi*60/4095, '-')
        plt.legend(('E(F) desired', 'E(F) measured'))
        plt.xlim((-30,70))
        plt.ylim((-90,10))
        plt.xlabel('$L_{ED}$ [mm]')
        plt.ylabel('$L_{FP}$ [mm]')

        plt.figure(3)
        plt.plot(np.array(self.t_w) - 5, np.array(self.wED)*g, '-')
        plt.plot(np.array(self.t_w) - 5, np.array(self.wFP)*g, '-')
        plt.plot(np.array(self.t_w) - 5, np.array(self.wIO)*g, '-')
        plt.xlim((0,5))
        plt.legend(('ED', 'FP', 'IO'))
        plt.xlabel('Time [s]')
        plt.ylabel('Force [N]')

        plt.figure(4)
        plt.plot(self.PIP, self.DIP, '-')
        plt.xlabel(r'$\theta_{PIP}$ [Deg]')
        plt.ylabel(r'$\theta_{DIP}$ [Deg]')

        plt.show()


    def change_params(self):
        start_time = time.time()
        temp_time = start_time

        self.DIP.append(self.calculate_angle(self.MP, self.DP, self.MP_index1, self.MP_index2, self.DP_index1, self.DP_index2) - self.DIP_angle)
        self.PIP.append(self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle)
        self.MCP.append(self.calculate_angle(self.MC, self.PP, self.MC_index1, self.MC_index2, self.PP_index1, self.PP_index2) - self.MCP_angle)

        self.LED_meas.append(0)
        self.LFP_meas.append(0)

        self.x_meas.append(self.DP.markers[self.DP_index1].position[0] - self.x0)
        self.z_meas.append(self.DP.markers[self.DP_index1].position[2] - self.z0)

        self.t_meas.append(0)
        self.PID_output = []

        #PID PARAMETERS
        error = 0
        previous_error = 0
        PID_P = 0
        PID_I = 0
        PID_D = 0
        self.PID_output.append(0)

        value = ser.readline()
        split = value.split(b'\t')
        self.wIO.append(0.25)
        self.wFP.append(0)
        self.wED.append(0)
        self.t_w.append(0)

        while time.time() - start_time < 45:
            current_time = time.time() - temp_time

            # Calculate new servo position
            index = np.argmin(abs(np.array(self.t_cal) - (current_time)))

            new_ED = self.init_ED + int(self.LED_cal[index])
            new_FP = self.init_FP + int(self.LFP_cal[index])

            # Write new servo position
            param_LED_meas = [DXL_LOBYTE(DXL_LOWORD(new_ED)), DXL_HIBYTE(DXL_LOWORD(new_ED)), DXL_LOBYTE(DXL_HIWORD(new_ED)), DXL_HIBYTE(DXL_HIWORD(new_ED))]
            param_LFP_meas = [DXL_LOBYTE(DXL_LOWORD(new_FP)), DXL_HIBYTE(DXL_LOWORD(new_FP)), DXL_LOBYTE(DXL_HIWORD(new_FP)), DXL_HIBYTE(DXL_HIWORD(new_FP))]
            groupSyncWrite_pos.addParam(self.default['ID_ED'], param_LED_meas)
            groupSyncWrite_pos.addParam(self.default['ID_FP'], param_LFP_meas)
            groupSyncWrite_pos.txPacket()
            groupSyncWrite_pos.clearParam()

            # Read servo position
            groupSyncRead_pos.txRxPacket()
            self.LED_meas.append((groupSyncRead_pos.getData(self.default['ID_ED'], ADDR_PRES_POSITION, LEN_4) - self.init_ED))
            self.LFP_meas.append((groupSyncRead_pos.getData(self.default['ID_FP'], ADDR_PRES_POSITION, LEN_4) - self.init_FP))
            self.t_meas.append(time.time() - start_time)

            # Append end-effector position
            self.x_meas.append(self.DP.markers[self.DP_index1].position[0] - self.x0)
            self.z_meas.append(self.DP.markers[self.DP_index1].position[2] - self.z0)

            # Measure IPJ coupling
            DIP = self.calculate_angle(self.MP, self.DP, self.MP_index1, self.MP_index2, self.DP_index1, self.DP_index2) - self.DIP_angle
            PIP = self.calculate_angle(self.PP, self.MP, self.PP_index1, self.PP_index2, self.MP_index1, self.MP_index2) - self.PIP_angle
            MCP = self.calculate_angle(self.MC, self.PP, self.MC_index1, self.MC_index2, self.PP_index1, self.PP_index2) - self.MCP_angle
            if (abs(self.DIP[-1] - DIP) <= 4 and abs(self.PIP[-1] - PIP) <= 4 and abs(self.MCP[-1] - MCP) <= 4):
                self.DIP.append(DIP)
                self.PIP.append(PIP)
                self.MCP.append(MCP)
                self.t_angles.append(time.time() - start_time)

            # Read tension in tendons
            value = ser.readline()
            split = value.split(b'\t')
            if (len(split) == 3):
                if (split[0] != b'' and split[1] != b'' and split[2] != b'\r\n'):
                    self.wIO.append((abs(int(split[0])) - self.V0[1])/(self.Vmax[1] - self.V0[1])*self.wmax)
                    self.wFP.append((abs(int(split[1])) - self.V0[2])/(self.Vmax[2] - self.V0[2])*self.wmax)
                    self.wED.append((abs(int(split[2])) - self.V0[3])/(self.Vmax[3] - self.V0[3])*self.wmax)
                    self.t_w.append(time.time() - start_time)

                    # Safety
                    if (self.wIO[-1]*g > 15 or self.wFP[-1]*g > 50 or self.wED[-1]*g > 50):
                        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                        packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

                    # PID controller
                    error =  self.default['setpoint_IO'] - self.wIO[-1]
                    dt = self.t_w[-1] - self.t_w[-2]

                    PID_P = self.default['Kp']*error

                    PID_I += error*dt

                    if (PID_I > 0.5):
                        PID_I = 0.5
                    elif (PID_I > 0.5):
                        PID_I = 0.5

                    PID_D = self.default['Kd']*(error - previous_error)/dt

                    PID = PID_P + self.default['Ki']*PID_I + PID_D

                    previous_error = error

                    PWM_IO = -self.default['PWM_IO']*PID

                    if PWM_IO < -500:
                      PWM_IO = -500

                    if PWM_IO > 500:
                      PWM_IO = 500
                    # if POS_IO < 1000:
                    #     POS_IO = 1000
                    #
                    # if POS_IO > 4000:
                    #     POS_IO = 4000

                    self.PID_output.append(PWM_IO)
                    #self.PID_output.append(POS_IO)

            # Write PWM IO
            param_PWM_IO = [DXL_LOBYTE(DXL_LOWORD(int(PWM_IO))), DXL_HIBYTE(DXL_LOWORD(int(PWM_IO)))]
            groupSyncWrite_PWM.addParam(self.default['ID_IO'], param_PWM_IO)
            groupSyncWrite_PWM.txPacket()
            groupSyncWrite_PWM.clearParam()

            ser.flushInput()
            ser.flushOutput()
            if (time.time() - temp_time > self.t_cal[-1]):
                temp_time = time.time()


        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_IO'], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

        packetHandler.write1ByteTxRx(portHandler, self.default['ID_ED'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        packetHandler.write1ByteTxRx(portHandler, self.default['ID_FP'], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

        self.LED_cal = np.array(self.LED_cal)
        self.LFP_cal = np.array(self.LFP_cal)

        self.LED_meas = np.array(self.LED_meas)
        self.LFP_meas = np.array(self.LFP_meas)


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
        if self.flag_traj == 0:
            ik = inverse_kinematics(self.x_cal, self.z_cal, self.t_cal)
            ik.load_data()
            ik.tendon_displ()
            self.LED_cal = ik.LED_cal
            self.LFP_cal = ik.LFP_cal
        else:
            pickle_in = open("dLED_hyper_%s.pickle"%sys.argv[2],"rb")
            self.LED_cal = pickle.load(pickle_in)

            pickle_in = open("dLFP_hyper_%s.pickle"%sys.argv[2],"rb")
            self.LFP_cal = pickle.load(pickle_in)

        self.port()

        self.restart()

        while(1):
            if ((int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_ED'], 123)[0])[-1]) == 1) and
                (int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_FP'], 123)[0])[-1]) == 1) and
                (int('{:b}'.format(packetHandler.read1ByteTxRx(portHandler, self.default['ID_IO'], 123)[0])[-1]) == 1)):
                break
        time.sleep(0.5)

        self.setup_OptiTrack()

        time.sleep(2)

        print("Start measurement.")

        self.setup()

        self.change_params()



if __name__ == '__main__':
    params = {'legend.fontsize': 'x-large',
          'figure.figsize': (13, 13),
         'axes.labelsize': 32,
         'axes.titlesize': 32,
         'xtick.labelsize': 27,
         'ytick.labelsize': 27}
    pylab.rcParams.update(params)

    model_config = {
        'ID_ED' : 3,
        'ID_FP' : 2,
        'ID_IO' : 4,
        'ID_OpenCM' : 200,
        'baudrate' : 1e6,
        'baudrate_val' : 3,
		'max_current': 30,
        'max_PWM' : 885,
        'PWM_IO' : 400,
        'restart_ED' : 1448,    # horizontal: 1484
        'restart_FP' : 1868,    # horizontal: 1990
        'restart_IO' : 2202,    # horizontal: 2234
        'setpoint_IO' : 0.200,
        'Kp' : 1.4,
        'Ki' : 15,
        'Kd' : 0.01
        }

    """
    0PIP: Kp: 1.3
          Ki: 15
          Kd: 0.001
          PWM_IO = 400


    0MCP: Kp : 1.8,
          Ki : 20,
          Kd : 0.001
          PWM_IO = 400


    grasp: Kp : 1.4,
           Ki : 5,
           Kd : 0.001
           PWM_IO = 400

    """


    pickle_in = open("x_traject_%s.pickle"%sys.argv[2],"rb")
    x_cal = pickle.load(pickle_in)

    pickle_in = open("z_traject_%s.pickle"%sys.argv[2],"rb")
    z_cal = pickle.load(pickle_in)

    pickle_in = open("time_traject_hyper_%s.pickle"%sys.argv[2],"rb")
    t_cal = pickle.load(pickle_in)

    exp = Experiment(model_config, x_cal, z_cal, 15*np.array(t_cal)/t_cal[-1], sys.argv[1])
    exp.run()
    #exp.plots()


    with open('%s_trigger_RL=1.15_v2_13-09.csv'%sys.argv[2], mode='w', newline='') as f:
        writer = csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        writer.writerow(['x_cal','z_cal','t_cal','x_meas','z_meas','t_meas','LED_cal','LFP_cal','LED_meas','LFP_meas','wED','wFP','wIO','t_w', 'MCP', 'PIP', 'DIP', 't_angles'])
        writer.writerow(np.array(exp.x_cal).astype(np.float))
        writer.writerow(np.array(exp.z_cal).astype(np.float))
        writer.writerow(np.array(exp.t_cal).astype(np.float))
        writer.writerow(np.array(exp.x_meas))
        writer.writerow(np.array(exp.z_meas))
        writer.writerow(np.array(exp.t_meas))
        writer.writerow(np.array(exp.LED_cal).astype(np.float))
        writer.writerow(np.array(exp.LFP_cal).astype(np.float))
        writer.writerow(np.array(exp.LED_meas))
        writer.writerow(np.array(exp.LFP_meas))
        writer.writerow(np.array(exp.wED))
        writer.writerow(np.array(exp.wFP))
        writer.writerow(np.array(exp.wIO))
        writer.writerow(np.array(exp.t_w))
        writer.writerow(np.array(exp.MCP))
        writer.writerow(np.array(exp.PIP))
        writer.writerow(np.array(exp.DIP))
        writer.writerow(np.array(exp.t_angles))
