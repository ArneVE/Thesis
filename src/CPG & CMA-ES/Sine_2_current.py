## Script that enables the control of 2 Dynamixels subjected to simple sine waves
## Dynamixel with ID = 2 represents the FP and Dyanmixel with ID = 3 represents the ED

import natnetclient as natnet
import matplotlib.pyplot as plt
import os
import csv
import xlsxwriter
import time
import pandas as pd
import numpy as np
import math

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
FP                     = 2                 # FP
ED                     = 3                 # ED
BAUDRATE                = 57600             # Dynamixel default baudrate : 57600
DEVICENAME              = 'COM3'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE           = 1                 # Value for enabling the torque
TORQUE_DISABLE          = 0                 # Value for disabling the torque
TORQUE_CONTROL_MODE     = 0                 # Value for torque control mode (operating mode)
VELOCITY_CONTROL_MODE   = 1                 # Value for velocity control mode (operating mode)

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite and GroupSyncRead instances
groupSyncWrite_cur = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_CUR, LEN_2)
groupSyncWrite_PWM = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_PWM, LEN_2)
groupSyncRead_vel = GroupSyncRead(portHandler, packetHandler, ADDR_PRES_VEL, LEN_4)
groupSyncRead_cur = GroupSyncRead(portHandler, packetHandler, ADDR_PRES_CUR, LEN_2)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()




################# SET BAUDRATE OPENCM9.04 #################
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 200, 12, 3)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Baudrate OpenCM9.04 changed.")




################# TORQUE DISABLE #################
# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, FP, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and torque is disabled" % FP)

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ED, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and torque is disabled" % ED)




################# SET OPERATING MODE #################
# Torque control mode Dynamixel#1
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, FP, ADDR_OPERATING_MODE, TORQUE_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and set in torque control mode" % FP)

# Torque control mode Dynamixel#2
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ED, ADDR_OPERATING_MODE, TORQUE_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and set in torque control mode" % ED)




################# SET CURRENT LIMIT #################
# Torque control mode Dynamixel#1
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, FP, ADDR_CUR_LIMIT, 200)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and current limit adjusted" % FP)

# Torque control mode Dynamixel#2
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ED, ADDR_CUR_LIMIT, 200)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and current limit adjusted" % ED)




################# ENABLE TORQUE #################
# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, FP, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and torque is enabled" % FP)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ED, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected and torque is enabled" % ED)




################# STORAGE CURRENT #################
# Add parameter storage for Dynamixel#1 present current value
addparam_result = groupSyncRead_cur.addParam(FP)
if addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % FP)
    quit()

# Add parameter storage for Dynamixel#2 present current value
addparam_result = groupSyncRead_cur.addParam(ED)
if addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % ED)
    quit()




################# EXTRA SETTINGS #################
packetHandler.write4ByteTxRx(portHandler, FP, 76, 0)
packetHandler.write4ByteTxRx(portHandler, ED, 76, 0)
packetHandler.write4ByteTxRx(portHandler, FP, 78, 0)
packetHandler.write4ByteTxRx(portHandler, ED, 78, 0)
packetHandler.write4ByteTxRx(portHandler, FP, 84, 0)
packetHandler.write4ByteTxRx(portHandler, ED, 84, 0)
packetHandler.write2ByteTxRx(portHandler, ED, ADDR_GOAL_PWM, 0)   # Set PWM (max velocity) of the ED
packetHandler.write2ByteTxRx(portHandler, FP, ADDR_GOAL_PWM, 0)   # Set PWM (max velocity) of the FP




ticks = []
data1 = []
## In the first experiment the ED will be subjected to a certain current curve and the FP will remain in its position
## In a second experiment the FP wil be subjected to a curve in counter phase
first_tick = time.time()
while 1:
    tick = time.time()
    t = tick - first_tick
    #current_ED = math.ceil(k*math.sin(t*math.pi + math.pi))
    current_FP = -math.ceil(100*math.sin(4*t*math.pi))

    if (np.sign(current_FP) == 1):
        k = 250
        l = 260
    else:
        k = 350
        l = 350

    #PWM_ED = k
    PWM_FP = l
    ticks.append(t)
    #packetHandler.write2ByteTxRx(portHandler, ED, ADDR_GOAL_PWM, PWM_ED)  # Change current of the extensor
    packetHandler.write2ByteTxRx(portHandler, FP, ADDR_GOAL_PWM, PWM_FP)  # Change current of the FP
    #packetHandler.write2ByteTxRx(portHandler, ED, ADDR_GOAL_CUR, current_ED)  # Change current of the extensor
    packetHandler.write2ByteTxRx(portHandler, FP, ADDR_GOAL_CUR, current_FP)  # Change current of the FP
    pres = packetHandler.read4ByteTxRx(portHandler, FP, ADDR_PRES_POSITION)
    if pres[0] > 2**12:
        adj = -(2**32 - pres[0])
    else:
        adj = pres[0]

    data1.append(adj)
    if (t > 6):
        print("Done.")
        packetHandler.write2ByteTxRx(portHandler, ED, ADDR_GOAL_CUR, 0)
        packetHandler.write2ByteTxRx(portHandler, FP, ADDR_GOAL_CUR, 0)
        break

# packetHandler.write1ByteTxRx(portHandler, FP, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
# packetHandler.write1ByteTxRx(portHandler, ED, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

plt.plot(ticks,data1)
plt.xlabel("Time [s]")
plt.ylabel("Velocity[units]")
plt.show()
