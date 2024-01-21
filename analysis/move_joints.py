from xarm.wrapper import XArmAPI
import time

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP)
arm.clean_error()
arm.motion_enable(enable=True)
arm.set_mode(mode=0)
arm.set_state(state=0)
time.sleep(1)

arm.set_servo_angle(angle=[0., -0.5, 0.5, 0., 0., 0.], is_radian=True, wait=True)

time.sleep(1)

arm.move_gohome(wait=True)

#arm.reset(wait=True)
#arm.motion_enable(enable=False)
#arm.disconnect()


