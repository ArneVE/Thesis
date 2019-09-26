## Script that disables the torque

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
ADDR_TORQUE_ENABLE      = 64

# Data Byte Length
LEN_1                   = 1
LEN_2                   = 2
LEN_4                   = 4

# Protocol version
PROTOCOL_VERSION        = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
FP                     = 2                 # FP
ED                     = 3                 # ED
BAUDRATE                = 1e6             # Dynamixel default baudrate : 57600
DEVICENAME              = 'COM3'            # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE           = 1                 # Value for enabling the torque
TORQUE_DISABLE          = 0                 # Value for disabling the torque
TORQUE_CONTROL_MODE     = 0                 # Value for torque control mode (operating mode)

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

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
