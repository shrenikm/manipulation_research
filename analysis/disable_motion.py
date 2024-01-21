from xarm.wrapper import XArmAPI

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP)
arm.motion_enable(enable=False)
arm.disconnect()


