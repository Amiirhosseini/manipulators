# Amirreza Hosseini 9820363

from re import X
from tkinter.tix import Y_REGION
import numpy as np
from math import cos, sin, pi
from scipy.optimize import fsolve


# declare theta1,theta3,d2 as variables

# theta1  = Symbol('theta1')
# d2  = Symbol('d2')
# theta3  = Symbol('theta3')

l2 = 1


# A basic code for matrix input from user
R = int(input("Enter the number of rows:"))
C = int(input("Enter the number of columns:"))

# Initialize matrix
matrix = []
print("Enter the entries rowwise:")

# For user input
for i in range(R):          # A for loop for row entries
    a = []
    for j in range(C):      # A for loop for column entries
        a.append(int(input()))
    matrix.append(a)


# define list of equations
# equation=[]

# add T_0_3[0,0]=matrix[0][0] to equation
# equation.append(T_0_3[0,0]matrix[0][0])


#X = Symbol('theta1')
# Y=Symbol('d2')
#Z = Symbol('theta3')


# s=solve([T_0_3[0,0]-matrix[0][0],T_0_3[1,0]-matrix[1][0],T_0_3[2,0]-matrix[2][0],T_0_3[3,0]-matrix[3][0]],(X,Y,Z))


def equation(variables):
    theta1, d2, theta3 = variables

    # intial T_0_3 4x4 matrix with theta1,theta3,d2
    T_0_3 = np.array([[cos(theta1)*cos(theta3), -cos(theta1)*sin(theta3), sin(theta1), (l2+d2)*sin(theta1)], [sin(theta1)*cos(
        theta3), -sin(theta1)*sin(theta3), -cos(theta1), -(l2+d2)*cos(theta1)], [sin(theta3), cos(theta3), 0, 0], [0, 0, 0, 1]])

    d = T_0_3[0][0]-matrix[0][0]

    return [T_0_3[0][2]-matrix[0][2], T_0_3[2][0]-matrix[2][0], T_0_3[1][3]-matrix[1][3]]


theta1, d2, theta3 = fsolve(equation, (0, 0, 0))


print("theta1=", theta1)
print("d2=", abs(d2))
print("theta3=", theta3)
