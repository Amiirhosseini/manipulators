from imports import *

# jacobian
def J(t11, t22, t33, t44):

    # convert to radian
    t11 = np.deg2rad(t11)
    t22 = np.deg2rad(t22)
    t33 = np.deg2rad(t33)
    t44 = np.deg2rad(t44)

    # # senti meter
    l0 = 11
    l1 = 6
    l2 = 12
    l3 = 12
    t1 = sp.Symbol('t1')
    t2 = sp.Symbol('t2')
    t3 = sp.Symbol('t3')
    t4 = sp.Symbol('t4')
    sym = [t1, t2, t3, t4]
    H = [[(-1*sp.sin(t2)*sp.sin(t3)*sp.cos(t1)+sp.cos(t1)*sp.cos(t2)*sp.cos(t3))*sp.cos(t4)+(-1*sp.sin(t2)*sp.cos(t1)*sp.cos(t3)-sp.sin(t3)*sp.cos(t1)*sp.cos(t2))*sp.sin(t4), -1*(-1*sp.sin(t2)*sp.sin(t3)*sp.cos(t1)+sp.cos(t1)*sp.cos(t2)*sp.cos(t3))*sp.sin(t4)+(-1*sp.sin(t2)*sp.cos(t1)*sp.cos(t3)-sp.sin(t3)*sp.cos(t1)*sp.cos(t2))*sp.cos(t4), sp.sin(t1), l2*sp.cos(t1)*sp.cos(t2)+l3*(-1*sp.sin(t2)*sp.sin(t3)*sp.cos(t1)+sp.cos(t1)*sp.cos(t2)*sp.cos(t3))],
         [(-1*sp.sin(t1)*sp.sin(t2)*sp.sin(t3)+sp.sin(t1)*sp.cos(t2)*sp.cos(t3))*sp.cos(t4)+(-1*sp.sin(t1)*sp.sin(t2)*sp.cos(t3) - sp.sin(t1)*sp.sin(t3)*sp.cos(t2))*sp.sin(t4),
             -1*(-1*sp.sin(t1)*sp.sin(t2)*sp.sin(t3)+sp.sin(t1)*sp.cos(t2)*sp.cos(t3))*sp.sin(t4) +
             (-1*sp.sin(t1)*sp.sin(t2)*sp.cos(t3) -
              sp.sin(t1)*sp.sin(t3)*sp.cos(t2))*sp.cos(t4),
             -1*sp.cos(t1), l2*sp.sin(t1)*sp.cos(t2)+l3*(-1*sp.sin(t1)*sp.sin(t2)*sp.sin(t3)+sp.sin(t1)*sp.cos(t2)*sp.cos(t3))],
         [(-1*sp.sin(t2)*sp.sin(t3)+sp.cos(t2)*sp.cos(t3))*sp.sin(t4)+(sp.sin(t2)*sp.cos(t3)+sp.sin(t3)*sp.cos(t2))*sp.cos(t4), (-1*sp.sin(t2)*sp.sin(t3)+sp.cos(t2)*sp.cos(t3))*sp.cos(t4)-(sp.sin(t2)*sp.cos(t3)+sp.sin(t3)*sp.cos(t2))*sp.sin(t4),
             0, l0+l1+l2*sp.sin(t2)+l3*(sp.sin(t2)*sp.cos(t3)+sp.sin(t3)*sp.cos(t2))],
         [0, 0, 0, 1]]
    end_effector = np.array([[15.5], [0], [0], [1]])
    T = np.dot(H, end_effector)

    # T = sp.Array(([- 12*sp.cos(t2)*sp.sin(t1) - 15*sp.cos(t2 + t3 + t4)*sp.sin(t1) - 12*sp.cos(t2 + t3)*sp.sin(t1), - 12*sp.cos(t1)*sp.sin(t2) - 15*sp.sin(t2 + t3 + t4)*sp.cos(t1) - 12*sp.sin(t2 + t3)*sp.cos(t1), - 15*sp.sin(t2 + t3 + t4)*sp.cos(t1) - 12*sp.sin(t2 + t3)*sp.cos(t1), -15*sp.sin(t2 + t3 + t4)*sp.cos(t1)],
    # [12*sp.cos(t1)*sp.cos(t2) + 15*sp.cos(t2 + t3 + t4)*sp.cos(t1) + 12*sp.cos(t2 + t3)*sp.cos(t1), - 12*sp.sin(t2 + t3)*sp.sin(t1) - 12*sp.sin(t1)*sp.sin(t2) -
    # 15*sp.sin(t2 + t3 + t4)*sp.sin(t1), - 12*sp.sin(t2 + t3)*sp.sin(t1) - 15*sp.sin(t2 + t3 + t4)*sp.sin(t1), -15*sp.sin(t2 + t3 + t4)*sp.sin(t1)],
    # [0,15*sp.cos(t2 + t3 + t4) + 12*sp.cos(t2 + t3) + 12*sp.cos(t2),15*sp.cos(t2 + t3 + t4) + 12*sp.cos(t2 + t3),15*sp.cos(t2 + t3 + t4)]))

    H_prim=[]

    for i in range(0, 3):
        for j in range(0, 4):
            # use sp.diff to find the derivative of the matrix
            H_prim.append(sp.diff(T[i][0], sym[j]))
            # subs the values of t1,t2,t3,t4
            H_prim[i*4+j]=H_prim[i*4 + j].subs([(t1, t11), (t2, t22), (t3, t33), (t4, t44)])

    H_prim=np.reshape(H_prim, (3, 4))

    return H_prim


def makeT(DH):
        T=np.array([[math.cos(DH[3]),                -math.sin(DH[3]),  0,  DH[1]],
                     [math.sin(DH[3])*math.cos(DH[0]), math.cos(DH[3]) * \
                               math.cos(DH[0]), -math.sin(DH[0]), -DH[2]*math.sin(DH[0])],
                     [math.sin(DH[3])*math.sin(DH[0]), math.cos(DH[3]) * \
                               math.sin(DH[0]),  math.cos(DH[0]),  DH[2]*math.cos(DH[0])],
                     [0,                                0,                                0,                1]])
        return T

def makeT_sp(DH):
        T=sp.Array([[sp.cos(DH[3]),                -sp.sin(DH[3]),  0,  DH[1]],
                     [sp.sin(DH[3])*sp.cos(DH[0]), sp.cos(DH[3]) * \
                             sp.cos(DH[0]), -sp.sin(DH[0]), -DH[2]*sp.sin(DH[0])],
                     [sp.sin(DH[3])*sp.sin(DH[0]), sp.cos(DH[3]) * \
                             sp.sin(DH[0]),  sp.cos(DH[0]),  DH[2]*sp.cos(DH[0])],
                     [0,                                0,                                0,                1]])
        return T


def forward_new(t11, t22, t33, t44):

    # convert to radian
    t11=np.deg2rad(t11)
    t22=np.deg2rad(t22)
    t33=np.deg2rad(t33)
    t44=np.deg2rad(t44)

    links=np.array([[0.0, 0.0, 17.0, t11], [math.radians(90), 0.0, 0.0, t22],
    [0, 12, 0.0, t33], [0, 12, 0, t44]])

    TT=np.eye(4)
    for i in range(0, 4):
        T=makeT(links[i])
        TT=TT.dot(T)

    # round the values of TT
    TT=np.round(TT, 3)

    return TT

# take long time to run this function
def J_new(t11, t22, t33, t44):
    #convert to radian
    t11=np.deg2rad(t11)
    t22=np.deg2rad(t22)
    t33=np.deg2rad(t33)
    t44=np.deg2rad(t44)

    t1=sp.Symbol('t1')
    t2=sp.Symbol('t2')
    t3=sp.Symbol('t3')
    t4=sp.Symbol('t4')
    sym=[t1, t2, t3, t4]

    links=sp.Array([[0.0, 0.0, 17.0, t1], [math.radians(90), 0.0, 0.0, t2],
    [0, 12, 0.0, t3], [0, 12, 0, t4]])

    TT=sp.eye(4)
    for i in range(0, 4):
        T=makeT_sp(links[i])
        TT=sp.tensorproduct(TT, T)

    H_prim=[]

    for i in range(0, 3):
        for j in range(0, 4):
            # use sp.diff to find the derivative of the matrix
            H_prim.append(sp.diff(TT[i][0], sym[j]))
            # subs the values of t1,t2,t3,t4
            H_prim[i*4+j]=H_prim[i*4 + j].subs([(t1, t11), (t2, t22), (t3, t33), (t4, t44)])

    H_prim=np.reshape(H_prim, (3, 4))

    return H_prim



# forward kinematic
def Hmatrix(t11, t22, t33, t44, dt1, dt2, dt3, dt4):

    dt1=np.deg2rad(dt1)
    dt2=np.deg2rad(dt2)
    dt3=np.deg2rad(dt3)
    dt4=np.deg2rad(dt4)


    H_prim=J(t11, t22, t33, t44)
    # H_prim=np.transpose(H_prim)
    dt=[dt1, dt2, dt3, dt4]
    dx=np.dot(H_prim, dt)

    return dx


# ----------------------------------------------------------------- #modified
# dar vaqe hamoon forward
# Hmatrix1Simple
def forward(t11, t22, t33, t44):

    # convert to radian
    t1=np.deg2rad(t11)
    t2=np.deg2rad(t22)
    t3=np.deg2rad(t33)
    t4=np.deg2rad(t44)

    l0=11
    l1=6
    l2=12
    l3=12
    sym=[t1, t2, t3, t4]
    H=[[(-1*np.sin(t2)*np.sin(t3)*np.cos(t1)+np.cos(t1)*np.cos(t2)*np.cos(t3))*np.cos(t4)+(-1*np.sin(t2)*np.cos(t1)*np.cos(t3)-np.sin(t3)*np.cos(t1)*np.cos(t2))*np.sin(t4), -1*(-1*np.sin(t2)*np.sin(t3)*np.cos(t1)+np.cos(t1)*np.cos(t2)*np.cos(t3))*np.sin(t4)+(-1*np.sin(t2)*np.cos(t1)*np.cos(t3)-np.sin(t3)*np.cos(t1)*np.cos(t2))*np.cos(t4), np.sin(t1), l2*np.cos(t1)*np.cos(t2)+l3*(-1*np.sin(t2)*np.sin(t3)*np.cos(t1)+np.cos(t1)*np.cos(t2)*np.cos(t3))],
         [(-1*np.sin(t1)*np.sin(t2)*np.sin(t3)+np.sin(t1)*np.cos(t2)*np.cos(t3))*np.cos(t4)+(-1*np.sin(t1)*np.sin(t2)*np.cos(t3) - np.sin(t1)*np.sin(t3)*np.cos(t2))*np.sin(t4),
             -1*(-1*np.sin(t1)*np.sin(t2)*np.sin(t3)+np.sin(t1)*np.cos(t2)*np.cos(t3))*np.sin(t4) +
             (-1*np.sin(t1)*np.sin(t2)*np.cos(t3) -
              np.sin(t1)*np.sin(t3)*np.cos(t2))*np.cos(t4),
             -1*np.cos(t1), l2*np.sin(t1)*np.cos(t2)+l3*(-1*np.sin(t1)*np.sin(t2)*np.sin(t3)+np.sin(t1)*np.cos(t2)*np.cos(t3))],
         [(-1*np.sin(t2)*np.sin(t3)+np.cos(t2)*np.cos(t3))*np.sin(t4)+(np.sin(t2)*np.cos(t3)+np.sin(t3)*np.cos(t2))*np.cos(t4), (-1*np.sin(t2)*np.sin(t3)+np.cos(t2)*np.cos(t3))*np.cos(t4)-(np.sin(t2)*np.cos(t3)+np.sin(t3)*np.cos(t2))*np.sin(t4),
             0, l0+l1+l2*np.sin(t2)+l3*(np.sin(t2)*np.cos(t3)+np.sin(t3)*np.cos(t2))],
         [0, 0, 0, 1]]
    end_effector= np.array([[15.5], [0], [0], [1]])
    T= np.dot(H, end_effector)

    return T


# inverse kinematics jacobian matrix principle
# inverse velocity kinematic
def inv_kin1(t11, t22, t33, t44, dx, dy, dz):

    j= J(t11, t22, t33, t44)
    # change j type from object to float64
    j= np.array(j, dtype=np.float64)
    J_transpose= np.transpose(j)
    # calculate J_star
    J_star= np.dot(J_transpose,np.linalg.inv(np.dot(j,J_transpose)))

    # J_star dot dx
    Q_dot= np.dot(J_star, [dx, dy, dz])

    return Q_dot



def inverse(x, y, z):

    d1= 17.6
    a2= 12
    a3= 24


    tetha1= math.atan2(y, x)
    r= math.sqrt(x**2 + y**2)
    s= z-d1
    D= (r**2 + s**2 - a2**2 - a3**2)/(2 * a2 * a3)
    # print(1-D**2)
    tetha3_pos= math.atan2(math.sqrt(1-D**2), D)
    tetha3_neg= math.atan2(-1*math.sqrt(1-D**2), D)

    #???
    if (tetha3_pos > -0.8 and tetha3_pos < 1.8):
        tetha3= tetha3_neg
    elif (tetha3_neg > -0.8 and tetha3_neg < 1.8):
        tetha3= tetha3_pos

    tetha2 = math.atan2(s, r) - math.atan2(a3 * math.sin(tetha3), a2+a3*math.cos(tetha3))

    tehta4= 0

    tetha1= np.rad2deg(tetha1)
    tetha2= np.rad2deg(tetha2)
    tetha3= np.rad2deg(tetha3)

    return tetha1, tetha2, tetha3, tehta4

