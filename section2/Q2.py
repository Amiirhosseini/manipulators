#Amirreza Hosseini 9820363
from AIUT_RoboticsToolbox.Toolbox import *

links = np.array([[0, 0, 0, 0, 0], [np.pi/2, 0, 0, 0, 1], [0, 0, 4, 0, 0]])
robot = SerialLink('Example 3.4', links)
T = robot.fkin([0, 2, 0])

#pirnt up to 2 fraction digits
print('T=', np.around(T, 2))

#print('T=',T)
robot.plot()


