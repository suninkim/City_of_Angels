import time

from pymycobot.mycobot import MyCobot

mc = MyCobot('COM9', 115200)
# mc = MyCobot('/dev/ttyAMA0' or '/dev/ttyUSB0', 1000000 or 115200)
# Users can modify and save it according to the version of the robot arm 
# and the actual serial number and baud rate.
# 用户自行根据机械臂的版本以及实际的串口号与波特率来进行修改并保存使用

mc.set_gripper_mode(0)
time.sleep(2)
mc.get_gripper_mode()

mode = mc.get_gripper_mode()
print(mode)

for count in range(3):
    mc.set_gripper_value(100, 50)
    time.sleep(2)
    mc.get_gripper_value()

    mc.set_gripper_value(30, 50)
    time.sleep(2)
    mc.get_gripper_value()

print(mode)
