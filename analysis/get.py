from xarm.wrapper import XArmAPI

ROBOT_IP = "192.168.1.178"

arm = XArmAPI(ROBOT_IP)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

print("=" * 50)
print("version:", arm.get_version())
print("state:", arm.get_state())
print("cmdnum:", arm.get_cmdnum())
print("err_warn_code:", arm.get_err_warn_code())
print("position(°):", arm.get_position(is_radian=False))
print("position(radian):", arm.get_position(is_radian=True))
print("angles(°):", arm.get_servo_angle(is_radian=False))
print("angles(radian):", arm.get_servo_angle(is_radian=True))
print("angles(°)(servo_id=1):", arm.get_servo_angle(servo_id=1, is_radian=False))
print("angles(radian)(servo_id=1):", arm.get_servo_angle(servo_id=1, is_radian=True))
print("Harmonic type: ", arm.get_harmonic_type())
print("Joints torque: ", arm.get_joints_torque())
print("Robot sn: ", arm.get_robot_sn())

arm.motion_enable(enable=False)
arm.disconnect()
