#Amirreza Hosseini 9820363
import numpy as np
import matplotlib.pyplot as plt
from pytransform3d.plot_utils import make_3d_axis
from pytransform3d.transform_manager import TransformManager
from scipy.spatial.transform import Rotation

#input matrix from user
T = np.array([[0.8,0.5,0],[0,1,-1],[1,0,2]])

#origin from user
origin = np.array([7,5,0])


#------------------------------------------
TA2U = np.eye(4)
R = Rotation.from_matrix(T).as_matrix()
TA2U[:3,:3] = R

#origin A in U
TA2U[:3,3] = origin
TU2A = np.linalg.inv(TA2U)

tm = TransformManager()
tm.add_transform("U","A",TU2A)
ax = make_3d_axis(6, 211)
ax = tm.plot_frames_in("U", ax=ax, s=4)
ax.view_init(30, 20)
plt.show()