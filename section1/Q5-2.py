#Amirreza Hosseini 9820363
from math import cos, sin
from cmath import acos
import numpy as np
from scipy.linalg import logm , expm 

#define r2angvec(r) function input 3*3 matrix and output theta and vector used in Q4.py
def r2angvec(r):
    #define theta and vector
    theta = acos((r[0][0]+r[1][1]+r[2][2]-1)/2).real
    v = np.array([r[2][1]-r[1][2], r[0][2]-r[2][0], r[1][0]-r[0][1]])/sin(theta)
    v=v/2
    #change theta from radian to degree
    theta = theta*(180/3.14)
    return theta, v


#assign k and theta from input
k = np.array([float(input("Enter the first row of the rotation matrix: ")), float(input("Enter the second row of the rotation matrix: ")), float(input("Enter the third row of the rotation matrix: "))])
theta = float(input("Enter the angle of rotation: "))

#change theta from degree to radian
theta = theta*(3.14/180)

K_matrix=np.array([[0,-k[2],k[1]],[k[2],0,-k[0]],[-k[1],k[0],0]])

#print K_matrix times 2
K_matrix_time2=K_matrix*K_matrix

print ("K_matrix_time2 is: ")
print ("{:>10.2f} {:>10.2f} {:>10.2f}".format(K_matrix_time2[0][0], K_matrix_time2[0][1], K_matrix_time2[0][2]))
print ("{:>10.2f} {:>10.2f} {:>10.2f}".format(K_matrix_time2[1][0], K_matrix_time2[1][1], K_matrix_time2[1][2]))
print ("{:>10.2f} {:>10.2f} {:>10.2f}".format(K_matrix_time2[2][0], K_matrix_time2[2][1], K_matrix_time2[2][2]))


e_k_theta= expm(K_matrix*theta)

#print e_k_theta in matrix form
print ("e_k_theta is: ")
print ("{:>10.2f} {:>10.2f} {:>10.2f}".format(e_k_theta[0][0], e_k_theta[0][1], e_k_theta[0][2]))
print ("{:>10.2f} {:>10.2f} {:>10.2f}".format(e_k_theta[1][0], e_k_theta[1][1], e_k_theta[1][2]))
print ("{:>10.2f} {:>10.2f} {:>10.2f}".format(e_k_theta[2][0], e_k_theta[2][1], e_k_theta[2][2]))

#reverse the way of calculation
[theta, k] = r2angvec(e_k_theta)

#print the result
print("The angle of rotation is: {:>10.2f}".format(theta))
print("The vector is: {:>10.2f} {:>10.2f} {:>10.2f}".format(k[0], k[1], k[2]))