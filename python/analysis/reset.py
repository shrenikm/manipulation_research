from xarm.wrapper import XArmAPI

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP)
arm.motion_enable(enable=True)
arm.set_state(state=0)

arm.reset(wait=True)
arm.stop_lite6_gripper()


