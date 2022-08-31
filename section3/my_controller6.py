from controller import Robot,Motor
print('my_controller')

robot = Robot()

number = robot.getNumberOfDevices()

base = robot.getDevice('base')
slider_joint = robot.getDevice('slider_joint')
head = robot.getDevice('head')

base.setVelocity(1)
head.setVelocity(1)

base.setPosition(6)
head.setPosition(32)

while robot.step(32) != -1:

    print(f'Number of Device: {n}')
    print(base.getTargetPosition())
    print(head.getTargetPosition())