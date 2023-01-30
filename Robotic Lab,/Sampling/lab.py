import os
import time
import numpy as np
import sympy as sp
from AIUT_RoboticsToolbox.Toolbox import *
import math
import keyboard

from dynamixel_sdk import *                 # Uses Dynamixel SDK library


# Control table address
PROTOCOL_VERSION = 2.0

BAUDRATE = 1000000

DXL_ID_array = [1, 2, 3, 4]

DEVICENAME = 'COM8'

# Control table address
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
# Refer to the Minimum Position Limit of product eManual
DXL_MINIMUM_POSITION_VALUE = 1500
# Refer to the Maximum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE = 2500


# Defines
TORQUE_ENABLE = 1     # Value for enabling the torque
TORQUE_DISABLE = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold
limits = [[1000, 3560], [1000, 3560], [1000, 3072], [1000, 3072]]


portHandler = PortHandler(DEVICENAME)

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

# #Reboot
# for i in range(0,4):
#     dxl_comm_result, dxl_error = packetHandler.reboot(portHandler, DXL_ID_array[i])
#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))

#     print("[ID:%03d] reboot Succeeded\n" % DXL_ID_array[i])

# print("------------------------------------------------------------")
# time.sleep(0.2)

# ping
for i in range(0, 4):
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(
        portHandler, DXL_ID_array[i])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" %
              (DXL_ID_array[i], dxl_model_number))

print("------------------------------------------------------------")
time.sleep(0.2)

# velocity
for i in range(0, 4):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID_array[i], 112, 15)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Velocity to 15 Succeeded." % (DXL_ID_array[i]))

print("------------------------------------------------------------")
time.sleep(0.2)

# Disable Dynamixel Torque
for i in range(0, 4):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID_array[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Disable Dynamixel Torque Succeeded." %
              (DXL_ID_array[i]))

print("------------------------------------------------------------")
time.sleep(0.2)

# upper/lower bound
for i in range(0, 4):
    # max
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID_array[i], 48, limits[i][1])
    # min
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID_array[i], 52, limits[i][0])
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] limit Succeeded." % (DXL_ID_array[i]))

print("------------------------------------------------------------")
time.sleep(0.2)

# Enable Dynamixel Torque
for i in range(0, 4):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID_array[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Enable Dynamixel Torque Succeeded." %
              (DXL_ID_array[i]))

print("------------------------------------------------------------")
time.sleep(0.2)

# Home
for i in range(0, 4):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID_array[i], ADDR_GOAL_POSITION, 2048)

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Home Dynamixel Torque Succeeded." % (DXL_ID_array[i]))

print("------------------------------------------------------------")


# Disable Dynamixel Torque
for i in range(0, 4):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID_array[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Disable Dynamixel Torque Succeeded." %
              (DXL_ID_array[i]))

print("------------------------------------------------------------")
time.sleep(0.2)

# create 2 damintion array
nemoone = [[0 for x in range(4)] for y in range(100000)]  # could not be enough
counter = 0


while 1:
    if keyboard.is_pressed('s') == True:
        break

# set timer
start = time.time()

while not keyboard.is_pressed('q'):

    for i in range(0, 4):
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, DXL_ID_array[i], ADDR_PRESENT_POSITION)

        # append nemoone[i][counter]
        nemoone[counter][i] = dxl_present_position

    counter += 1

# stop timer
end = time.time()

frequency = counter/(end-start)
print(frequency)

# Enable Dynamixel Torque
for i in range(0, 4):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, DXL_ID_array[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Enable Dynamixel Torque Succeeded." %
              (DXL_ID_array[i]))

print("------------------------------------------------------------")
time.sleep(0.2)


c = counter
counter = 0
while counter != c:

    for i in range(0, 4):
        # Write goal position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, DXL_ID_array[i], ADDR_GOAL_POSITION, nemoone[counter][i])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    counter += 1


# Close port
portHandler.closePort()
