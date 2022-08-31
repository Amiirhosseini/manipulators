#Amirreza Hosseini 9820363
from math import cos, sin

#define angvec2r(theta,v) function input theta and vector v and give general 3*3 rotation matrix
def angvec2r(theta,v):
    #define the matrix
    rot = [[v[0]*v[0]*(1-cos(theta))+cos(theta), v[0]*v[1]*(1-cos(theta))-v[2]*sin(theta), v[0]*v[2]*(1-cos(theta))+v[1]*sin(theta)], 
              [v[1]*v[0]*(1-cos(theta))+v[2]*sin(theta), v[1]*v[1]*(1-cos(theta))+cos(theta), v[1]*v[2]*(1-cos(theta))-v[0]*sin(theta)],
                [v[2]*v[0]*(1-cos(theta))-v[1]*sin(theta), v[2]*v[1]*(1-cos(theta))+v[0]*sin(theta), v[2]*v[2]*(1-cos(theta))+cos(theta)]]
    return rot

#test the function  
theta=float(input("Enter the angle of rotation: "))
v=[float(input("Enter the x component of the vector: ")), float(input("Enter the y component of the vector: ")), float(input("Enter the z component of the vector: "))]

#change theta from degree to radian
theta=theta*(3.14/180)

#show the output in matrix format
print("The rotation matrix is: ")
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(angvec2r(theta,v)[0][0], angvec2r(theta,v)[0][1], angvec2r(theta,v)[0][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(angvec2r(theta,v)[1][0], angvec2r(theta,v)[1][1], angvec2r(theta,v)[1][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(angvec2r(theta,v)[2][0], angvec2r(theta,v)[2][1], angvec2r(theta,v)[2][2]))