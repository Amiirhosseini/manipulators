#Amirreza Hosseini 9820363
from math import cos, sin

#define Z'Y'Z' eul2r(alpha, beta, gamma) function
def eul2r(alpha, beta, gamma):
       ca = cos(alpha)
       cb = cos(beta)
       cg = cos(gamma)
       sa = sin(alpha)
       sb= sin(beta)
       sg = sin(gamma)
       R = [[ca*cb*cg-sa*sg, -ca*cb*sg-sa*cg, ca*sb],
                [sa*cb*cg+ca*sg, -sa*cb*sg+ca*cg, sa*sb],
                     [-sb*cg, sb*sg, cb]]
       return R

 

#test the function
alpha=float(input("Enter the angle of rotation about the x axis: "))
beta=float(input("Enter the angle of rotation about the y axis: "))
gamma=float(input("Enter the angle of rotation about the z axis: "))

#change alpha, beta, gamma from degree to radian
alpha=alpha*(3.14/180)
beta=beta*(3.14/180)
gamma=gamma*(3.14/180)

#print the output in matrix format
print("The rotation matrix is: ")
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(eul2r(alpha, beta, gamma)[0][0], eul2r(alpha, beta, gamma)[0][1], eul2r(alpha, beta, gamma)[0][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(eul2r(alpha, beta, gamma)[1][0], eul2r(alpha, beta, gamma)[1][1], eul2r(alpha, beta, gamma)[1][2]))
print("{:>10.2f} {:>10.2f} {:>10.2f}".format(eul2r(alpha, beta, gamma)[2][0], eul2r(alpha, beta, gamma)[2][1], eul2r(alpha, beta, gamma)[2][2]))
