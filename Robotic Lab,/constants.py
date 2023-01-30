# --------------------------------constants--------------------------------
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
limits = [[1058, 3038], [1553, 3533], [2543, 1058], [1553, 3038]]
# ----------------------------end of constants------------------------------