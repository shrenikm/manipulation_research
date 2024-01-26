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

arm.reset(wait=True)

arm.set_position(x=300, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=100, is_radian=False, wait=True)
arm.close_lite6_gripper()
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(x=300, y=200, z=250, roll=-180, pitch=0, yaw=0, speed=200, is_radian=False, wait=True)
arm.open_lite6_gripper()
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(x=300, y=-200, z=150, roll=-180, pitch=0, yaw=0, speed=200, is_radian=False, wait=True)
arm.close_lite6_gripper()
print(arm.get_position(), arm.get_position(is_radian=False))
arm.set_position(x=300, y=0, z=250, roll=-180, pitch=0, yaw=0, speed=100, is_radian=False, wait=True)
arm.open_lite6_gripper()
print(arm.get_position(), arm.get_position(is_radian=False))


arm.reset(wait=True)



