#Amirreza Hosseini 9820363
from AIUT_RoboticsToolbox.Toolbox import *

links = np.array([[0, 0, 0, 0, 0], [0, 0, 0, 0, 1], [np.pi/2, 0, 0, 0, 1]])
robot = SerialLink('FIGURE 3.39', links)
T = robot.fkin([0, 5, 4])

#pirnt up to 2 fraction digits
print('T=', np.around(T, 2))

robot.plot()
