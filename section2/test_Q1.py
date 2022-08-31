#Amirreza Hosseini 9820363
import numpy as np
import math

firstRow = np.array([0, 0, 0, 0])
secondRow = np.array([0, 0, 5, 0])
thirdRow = np.array([np.pi/2, 0, 4, 0])

T_0_1 = np.array([[math.cos(firstRow[3]),                -math.sin(firstRow[3]),  0,  firstRow[1]],
                  [math.sin(firstRow[3])*math.cos(firstRow[0]), math.cos(firstRow[3]) *
                   math.cos(firstRow[0]), -math.sin(firstRow[0]), -firstRow[2]*math.sin(firstRow[0])],
                  [math.sin(firstRow[3])*math.sin(firstRow[0]), math.cos(firstRow[3]) *
                   math.sin(firstRow[0]),  math.cos(firstRow[0]),  firstRow[2]*math.cos(firstRow[0])],
                  [0,                                0,                                0,                1]])

T_1_2 = np.array([[math.cos(secondRow[3]),                -math.sin(secondRow[3]),  0,  secondRow[1]],
                  [math.sin(secondRow[3])*math.cos(secondRow[0]), math.cos(secondRow[3]) *
                   math.cos(secondRow[0]), -math.sin(secondRow[0]), -secondRow[2]*math.sin(secondRow[0])],
                  [math.sin(secondRow[3])*math.sin(secondRow[0]), math.cos(secondRow[3]) *
                   math.sin(secondRow[0]),  math.cos(secondRow[0]),  secondRow[2]*math.cos(secondRow[0])],
                  [0,                                0,                                0,                1]])

T_2_3 = np.array([[math.cos(thirdRow[3]),                -math.sin(thirdRow[3]),  0,  thirdRow[1]],
                  [math.sin(thirdRow[3])*math.cos(thirdRow[0]), math.cos(thirdRow[3]) *
                   math.cos(thirdRow[0]), -math.sin(thirdRow[0]), -thirdRow[2]*math.sin(thirdRow[0])],
                  [math.sin(thirdRow[3])*math.sin(thirdRow[0]), math.cos(thirdRow[3]) *
                   math.sin(thirdRow[0]),  math.cos(thirdRow[0]),  thirdRow[2]*math.cos(thirdRow[0])],
                  [0,                                0,                                0,                1]])


# T_0_1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
# T_1_2 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 5], [0, 0, 0, 1]])
# T_2_3 = np.array([[1, 0, 0, 0], [0, 0, -1, -4], [0, 1, 0, 0], [0, 0, 0, 1]])

#multiplication of transformation matrices
T_0_2 = np.dot(T_0_1, T_1_2)
T_0_3 = np.dot(T_0_2, T_2_3)

#print(T_0_3)
print('T=', np.around(T_0_3, 2))
