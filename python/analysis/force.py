from xarm.wrapper import XArmAPI
import time

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP)
arm.clean_error()
arm.clean_warn()

arm.motion_enable(enable=True)

arm.ft_sensor_enable(True)
arm.ft_sensor_set_zero()
time.sleep(0.2)

print(arm.get_ft_sensor_data())

arm.ft_sensor_app_set(1)
arm.set_mode(mode=0)

arm.ft_sensor_app_set(0)
arm.ft_sensor_enable(0)
arm.disconnect()
