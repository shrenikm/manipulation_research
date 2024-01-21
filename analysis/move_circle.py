import numpy as np
from xarm.wrapper import XArmAPI
import time

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP)
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(mode=0)
arm.set_state(state=0)
time.sleep(1)

poses = [
    [200,  0,   200, -180, 0, 0],
    [200,  100, 200, -180, 0, 0],
    [300,  100, 200, -180, 0, 0],
    [300, -100, 200, -180, 0, 0],
    [200,  0,   300, -180, 0, 0]
]

ret = arm.set_position(*poses[0], speed=50, mvacc=100, wait=False)
print('set_position, ret: {}'.format(ret))

ret = arm.move_circle(pose1=poses[1], pose2=poses[2], percent=50, speed=200, mvacc=1000, wait=True)
print('move_circle, ret: {}'.format(ret))

ret = arm.move_circle(pose1=poses[3], pose2=poses[4], percent=200, speed=200, mvacc=1000, wait=True)
print('move_circle, ret: {}'.format(ret))

arm.move_gohome()




