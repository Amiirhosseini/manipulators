#Amirreza Hosseini 9820363
import numpy as np
from scipy.linalg import logm , expm 

#find unit quaternion from rotation matrix
def unit_quaternion(R):
    q = np.zeros(4)
    q[0] = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2])/2
    q[1] = (R[2,1] - R[1,2])/(4*q[0])
    q[2] = (R[0,2] - R[2,0])/(4*q[0])
    q[3] = (R[1,0] - R[0,1])/(4*q[0])
    return q

#find equivalent axis K and angle theta from unit quaternion
def axis_angle(q):
    theta = 2*np.arccos(q[0])
    K = np.zeros(3)
    K[0] = q[1]/np.sin(theta/2)
    K[1] = q[2]/np.sin(theta/2)
    K[2] = q[3]/np.sin(theta/2)
    return K,theta

#test both functions
def test():
    #R= exact rotation matrix
    R = np.array([[0.5,0.5,0.5],[0.5,0.5,-0.5],[0.5,-0.5,0.5]])
    q = unit_quaternion(R)
    K,theta = axis_angle(q)
    #print q,k,theta    
    print("q = ",q)
    print("K = ",K)
    print("theta = ",theta)
    
#call test function
test()