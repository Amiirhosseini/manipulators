#Amirreza Hosseini
#Robotics Lab.
#2023

from functions import *
from constants import *
import keyboard

DXL_ID_array = [1, 2, 3, 4]

pos = [0, 0, 0, 0, 0]
# set position

#just go to thetas
def set_pos(t1, t2, t3, t4):
    pos[1] = 11*t1+2048
    pos[2] = 11*t2+2048
    pos[3] = -11*t3+2048
    pos[4] = 11*t4+2048
    
#motor to theta
def motor_to_theta(motor1,motor2,motor3,motor4):
    theta1 = (motor1-2048)/11
    theta2 = (motor2-2048)/11
    theta3 = -(motor3-2048)/11
    theta4 = (motor4-2048)/11
    return theta1,theta2,theta3,theta4

def two_point(t1, t2, t3, t4, x, y, z, error, alpha,threshold):

    x_e = [x, y, z]

    T_c = [t1, t2, t3, t4]

    while (error > threshold):
        T = forward(T_c[0], T_c[1], T_c[2], T_c[3])
        T = T[0:3]
        T = np.reshape(T, (1, 3))
        T = T[0]
        # change list to array
        T = np.array(T, dtype=np.float64)
        x_e = np.array(x_e)  # noghte nahaei

        delta_x = alpha*((x_e - T) / np.linalg.norm(x_e - T))
        T_c = inv_kin1(t1, t2, t3, t4, delta_x[0], delta_x[1], delta_x[2])+T_c
        set_pos(int(T_c[0]), int(T_c[1]), int(T_c[2]), int(T_c[3]))

        for i in range(0, 4):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_array[i], ADDR_GOAL_POSITION, pos[i+1])
            print(dxl_error)

        # sleep inja
        time.sleep(0.020)

        X_c = forward(T_c[0], T_c[1], T_c[2], T_c[3])

        X_c = X_c[0:3]
        X_c = np.array(X_c, dtype=np.float64)
        error = np.linalg.norm(x_e - X_c)


def two_point2(t1, t2, t3, t4, x, y, z, error, alpha,threshold):
    
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
        
        
    motor=[0,0,0,0]
    
    while not keyboard.is_pressed('q'):
        for i in range(0, 4):
            # Read present position
            dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
                portHandler, DXL_ID_array[i], ADDR_PRESENT_POSITION)
            
            motor[i]=dxl_present_position
        
    t1,t2,t3,t4=motor_to_theta(motor[0],motor[1],motor[2],motor[3])
    
    T=forward(t1,t2,t3,t4)

    x_e = [x, y, z]

    T_c = [t1, t2, t3, t4]

    while (error > threshold):
        T = forward(T_c[0], T_c[1], T_c[2], T_c[3])
        T = T[0:3]
        T = np.reshape(T, (1, 3))
        T = T[0]
        # change list to array
        T = np.array(T, dtype=np.float64)
        x_e = np.array(x_e)  # noghte nahaei

        delta_x = alpha*((x_e - T) / np.linalg.norm(x_e - T))
        T_c = inv_kin1(t1, t2, t3, t4, delta_x[0], delta_x[1], delta_x[2])+T_c
        set_pos(int(T_c[0]), int(T_c[1]), int(T_c[2]), int(T_c[3]))

        for i in range(0, 4):
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_array[i], ADDR_GOAL_POSITION, pos[i+1])
            print(dxl_error)

        # sleep inja
        time.sleep(0.020)

        X_c = forward(T_c[0], T_c[1], T_c[2], T_c[3])

        X_c = X_c[0:3]
        X_c = np.array(X_c, dtype=np.float64)
        error = np.linalg.norm(x_e - X_c)

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
    
    if(i==0):
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
        portHandler, DXL_ID_array[i], 112, 15)
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, DXL_ID_array[i], 112, 20)
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

# -----------------------------------------------------
# # # #forward addi\
#down limit
# t1=-90
# t2=-45
# t3=-45
# t4=-45

#upper limit
# t1=90
# t2=135
# t3=90
# t4=90

# #1
t1=30
t2=-10
t3=+60
t4=-40

# #2
t1=-10
t2=20
t3=-30
t4=+50

# #3
t1=-45
t2=30
t3=30
t4=20



# #set position
set_pos(t1,t2,t3,t4)

for i in range(0,4):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_array[i], ADDR_GOAL_POSITION, pos[i+1])

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Enable Dynamixel Torque Succeeded." % (DXL_ID_array[i]))

print("------------------------------------------------------------")
time.sleep(0.2)

# #forward addi
T=forward(t1,t2,t3,t4)

# #---------------------------------------------
# # #inverse
tetha1,tetha2,tetha3,tehta4 = inverse(+25,-22,25)


#set position
set_pos(int(tetha1),int(tetha2),int(tetha3),tehta4)

for i in range(0,4):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_array[i], ADDR_GOAL_POSITION, pos[i+1])

    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("[ID:%03d] Enable Dynamixel Torque Succeeded." % (DXL_ID_array[i]))


# # ----------------------------------------------------
# # jacobian
#Hmatrix(t1,t2,t3,t4,0,0,1,0)

#J(45,45,45,45)

#----------------
# #forward addi
#forward_new(t1,t2,t3,t4)



# # inverse jacobian
# T_prim1 = 45
# T_prim2 = 15
# T_prim3 = 30
# T_prim4 = 20

# q_dot = inv_kin1(T_prim1, T_prim2, T_prim3, T_prim4, 1, 1, 0)

# # round q_dot
# for i in range(0, 4):
#     q_dot[i] = round(q_dot[i], 2)


# # set position
# set_pos(int(T_prim1), int(T_prim2), int(T_prim3), int(T_prim4))

# for i in range(0, 4):
#     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID_array[i], ADDR_GOAL_POSITION, pos[i+1])

#     if dxl_comm_result != COMM_SUCCESS:
#         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#     elif dxl_error != 0:
#         print("%s" % packetHandler.getRxPacketError(dxl_error))
#     else:
#         print("[ID:%03d] Enable Dynamixel Torque Succeeded." %
#               (DXL_ID_array[i]))

# --------------------------------------------------------------------------------------------------------
# call two point movement
two_point2(45, 45, 45, 45, 19.5, 19.5, 45,7,5,5)


# #------------------------------use Palhang---------------------------------------------
# # Ti
# links = np.array([[0.0, 0.0, 17.6, 0.0, 0],
#                   [np.pi/2, 0.0, 0.0, 0.0, 0], [0.0, 12, 0.0, 0.0, 0], [0.0, 12, 0.0, 0.0, 0]])

# robot = SerialLink('Test Lab', links)

# H = robot.fkin([np.deg2rad(t1), np.deg2rad(t2),
#                np.deg2rad(t3), np.deg2rad(t4)])
# print('H=', H)
# # end_effector=np.array([[1,0,0,15.5],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
# end_effector = np.array([[15.5], [0], [0], [1]])
# print('end=', end_effector)
# T = np.dot(H, end_effector)
# print('T=', T)
# #----------------------------------------------end of Palhang---------------------------------------------

# Close port
portHandler.closePort()
