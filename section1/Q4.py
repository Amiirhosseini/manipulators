#Amirreza Hosseini 9820363
from cmath import acos
from math import cos, sin
from numpy import angle
import numpy as np

#define r2angvec(r) function input 3*3 matrix and output theta and vector
def r2angvec(r):
    #define theta and vector
    theta = acos((r[0][0]+r[1][1]+r[2][2]-1)/2).real

    v = np.array([r[2][1]-r[1][2], r[0][2]-r[2][0], r[1][0]-r[0][1]])/sin(theta)
    v=v/2
    
    #change theta from radian to degree
    theta = theta*(180/3.14)
    
    return theta, v

#test the r2angvec function
r = np.array([[0.933,0.067,0.354],[0.067,0.933,-0.354],[-0.354,0.354,0.866]])

#show the output theta and vector
print("The angle of rotation is: {:>10.2f}".format(r2angvec(r)[0]))
print("The vector is: {:>10.2f} {:>10.2f} {:>10.2f}".format(r2angvec(r)[1][0], r2angvec(r)[1][1], r2angvec(r)[1][2]))