#Amirreza Hosseini  9820363
from math import cos, sin
from numpy import angle

#define rotx(ang) function
def rotx(ang):
    #define the matrix
    rotx = [[1, 0, 0], [0, cos(ang), -sin(ang)], [0, sin(ang), cos(ang)]]
    return rotx 

#define roty(ang) function
def roty(ang):  
    #define the matrix
    roty = [[cos(ang), 0, sin(ang)], [0, 1, 0], [-sin(ang), 0, cos(ang)]]
    return roty

#define rotz(ang) function
def rotz(ang):
    #define the matrix
    rotz = [[cos(ang), -sin(ang), 0], [sin(ang), cos(ang), 0], [0, 0, 1]]
    return rotz


angle=float(input("Enter the angle of rotation: "))

#change angle from degree to radian
angle=angle*(3.14/180)


print("The rotation matrix is: ")
#print the x rotation matrix with text with matrix format
print("x rotation matrix: ")
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(rotx(angle)[0][0], rotx(angle)[0][1], rotx(angle)[0][2])) 
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(rotx(angle)[1][0], rotx(angle)[1][1], rotx(angle)[1][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(rotx(angle)[2][0], rotx(angle)[2][1], rotx(angle)[2][2]))
#print the y rotation matrix with text formatting
print("y rotation matrix: ")
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(roty(angle)[0][0], roty(angle)[0][1], roty(angle)[0][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(roty(angle)[1][0], roty(angle)[1][1], roty(angle)[1][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(roty(angle)[2][0], roty(angle)[2][1], roty(angle)[2][2]))
#print the z rotation matrix with text formatting
print("z rotation matrix: ")
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(rotz(angle)[0][0], rotz(angle)[0][1], rotz(angle)[0][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(rotz(angle)[1][0], rotz(angle)[1][1], rotz(angle)[1][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(rotz(angle)[2][0], rotz(angle)[2][1], rotz(angle)[2][2]))


