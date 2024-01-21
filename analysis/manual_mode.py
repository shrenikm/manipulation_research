import time
from xarm.wrapper import XArmAPI

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP, is_radian=True)
arm.clean_error()
arm.motion_enable(enable=True)
#arm.set_mode(0)
#arm.set_state(state=0)

# Turn on manual mode before recording
arm.set_mode(2)
arm.set_state(0)

print("Manual start!")
arm.start_record_trajectory()

time.sleep(20)

arm.stop_record_trajectory()
print("Manual end!")
#arm.motion_enable(enable=False)
#arm.disconnect()
