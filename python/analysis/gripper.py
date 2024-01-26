from xarm.wrapper import XArmAPI
import time

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP)
arm.clean_error()

arm.motion_enable(enable=True)
arm.set_mode(mode=0)
arm.set_state(state=0)
time.sleep(1)

#arm.set_gripper_mode(0)
#arm.set_gripper_enable(True)

for _ in range(10):
    arm.close_lite6_gripper()
    time.sleep(3)
    arm.open_lite6_gripper()
    time.sleep(3)
arm.stop_lite6_gripper()

