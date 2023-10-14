import json
import math
import os
import threading
import time
import sys
import cv2
import atexit
import signal
import numpy as np
import pyrealsense2 as rs
import serial
import serial.tools.list_ports
from pymycobot import PI_BAUD, PI_PORT
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot

from utils import utils

PI = math.pi
def rad2deg(rad):
    return rad * 180.0 / PI

def deg2rad(deg):
    return deg * PI / 180.0


def convert_os_command(command):
    return command


class RealTaskBase():
    def __init__(self, port='/dev/ttyAMA0', baudrate="115200", timeout=0.1, debug=False):
        self.robot = MyCobot(port, baudrate, debug=debug)

        self.homing_angle = np.array([-0.2, 0.45, 0.85, 0.27, -1.47, 1.37]) # change to yaml
        self.terminate_angle = np.array([0.0, 1.67, 1.0, -1.1, 0.0, 0.0]) # change to yaml

        self.robot.set_gripper_mode(0)
        self.gripper_mode = self.robot.get_gripper_mode()

        self.is_calibrated = False # change to yaml
        self.calibration_num = False

        if not self.is_calibrated:
            need_calibration = str(input("Robot is not calibrated. Do you want to perform calibration? [Y/N]")) 
            if need_calibration == "Y" or need_calibration == "y":
                for _ in range(6):
                    self.calibration_robot()
                    time.sleep(1)
            else:
                a=1
                # raise Exception("Cannot proceed without calibrating the robot first.")

        self.setup_camera()
        signal.signal(signal.SIGINT, self.signal_handler)
        atexit.register(self.shut_down)

        self.set_color(0,255,255)
        self.move(self.homing_angle)
        self.gripper_control(True)
        time.sleep(3)
        self.update_info()
        self.set_color(0,255,255)
        print("Initialize real robot env done!")

    ###########################
    ######    Homing     ######
    ###########################

    def calibration_robot(self):
        """Calibration button click event.

        Click to calibrate one motor at a time and calibrate in turn. After all
        calibration, resume initialization.
        """
        if not self.has_mycobot():
            return

        if not self.calibration_num:
            self.calibration_num = 0

        self.calibration_num += 1

        self.robot.set_servo_calibration(self.calibration_num)
        time.sleep(0.1)
        self.robot.focus_servo(self.calibration_num)
        time.sleep(0.5)
        pos=self.robot.get_angles()
        print("Calibration of motor "+str(self.calibration_num) + " completed.")

        if self.calibration_num == 6:
            print("Calibration is succeessed!")
            self.calibration_num = None
            self.calibration_test()

    def calibration_test(self):
        if str(input("Robot is calibrated. Do you want to perform calibration test? [Y/N]")) != "Y":
            return
        else:
            print("Starting calibration test.")
            time.sleep(0.5)
            angles = [0, 0, 0, 0, 0, 0]
            test_angle = [-20, 20, 0]
            for i in range(6):
                for j in range(3):
                    angles[i] = test_angle[j]
                    self.robot.send_angles(angles, 30)
                    time.sleep(2)
            self.joint_angle = [0, 0, 0, 0, 0, 0]
            self.gripper_control(True)
            self.gripper_control(False)
            print("Calibration test finisihed!")

        
    ###########################
    #####  Basic Movement  ####
    ###########################

    def move(self, command, speed=30, coordinate="js"):
        self.update_info()
        
        # is_motor_connected = self.check_servo()
        if coordinate == "js":
            self.robot.sync_send_angles(rad2deg(command), speed, timeout=0.001)
        elif coordinate == "os":
            self.robot.sync_send_coords(convert_os_command(command), speed, timeout=2)


    def gripper_control(self, open):
        if open:
            self.robot.set_gripper_value(100, 50)
        else:
            self.robot.set_gripper_value(20, 50)
        time.sleep(1)
        self.gripper_value = self.robot.get_gripper_value()

    def move_to_zero(self):
        self.move(np.array([0,0,0,0,0,0]))
        self.gripper_control(False)

    def step(self, action, relative=True):
        self.update_info()
        if relative:
            action_command = self.joint_angle.copy() + action
        else:
            action_command = action
        self.move(action_command)
        self.update_info()
        return self.joint_angle

    ###########################
    ###### Record & Play ######
    ###########################
    def record(self):
        self.record_list = []
        self.recording = True
        self.robot.set_fresh_mode(0)
        def _record():
            start_t = time.time()

            while self.recording:
                angles = self.robot.get_encoders()
                if angles:
                    self.record_list.append(angles)
                    time.sleep(0.1)
                    print("\r {}".format(time.time() - start_t), end="")

        print("Start recording.")
        self.record_t = threading.Thread(target=_record, daemon=True)
        self.record_t.start()
    
    def stop_record(self):
        if self.recording:
            self.recording = False
            self.record_t.join()
            print("Stop record")

    def play(self):
        print("Start play")
        for angles in self.record_list:
            # print(angles)
            self.robot.set_encoders(angles, 80)
            time.sleep(0.1)
        print("Finish play")

    def loop_play(self):
        self.playing = True

        def _loop():
            len_ = len(self.record_list)
            i = 0
            while self.playing:
                idx_ = i % len_
                i += 1
                self.robot.set_encoders(self.record_list[idx_], 80)
                time.sleep(0.1)

        print("Start loop play.")
        self.play_t = threading.Thread(target=_loop, daemon=True)
        self.play_t.start()

    def stop_loop_play(self):
        if self.playing:
            self.playing = False
            self.play_t.join()
            print("Stop loop play.")

    def save_to_local(self):
        if not self.record_list:
            print("No data should save.")
            return

        with open(os.path.dirname(__file__) + "/record.txt", "w") as f:
            json.dump(self.record_list, f, indent=2)
            print("save dir:  {}".format(os.path.dirname(__file__)))

    def load_from_local(self):

        with open(os.path.dirname(__file__) + "/record.txt", "r") as f:
            try:
                data = json.load(f)
                self.record_list = data
                print("Load data success.")
            except Exception:
                print("Error: invalid data.")

    ###########################
    ######  Data Process ######
    ###########################

    def get_rgb_image(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture image.")
        
        cv2.imwrite("image.png",frame)
        return frame

    def get_depth_image(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image

    def get_init_state(self):
        self.update_info()
        return self.joint_angle
    
    def get_angle(self):
        return self.joint_angle

    def get_coord(self):
        return self.coord

    def get_speed(self):
        return self.joint_speed
    
    def get_is_joint_moving(self):
        return self.is_joint_moving
    
    def get_is_gripper_moving(self):
        return self.is_gripper_moving

    ###########################
    ######   Connection  ######
    ###########################

    def setup_camera(self):

        camera_num = 0 
        self.cap = cv2.VideoCapture(camera_num)

        if not self.cap.isOpened():
            print("Failed to open camera.")
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def set_color(self, r,g,b):
        self.robot.set_color(r,g,b)

    def has_mycobot(self):
        """Check whether it is connected on mycobot"""
        if not self.robot:
            print("mycobot has not been connected yet!!!")
            return False
        return True

    def check_servo(self):
        res = []
        for i in range(1,8):
            _data = self.robot.get_servo_data(i , 5)
            time.sleep(0.02)
            if _data != i:
                res.append(i)
        if res:
            print("Motor {} is not connected!".format(res))
            return False
        else:
            return True

    def signal_handler(self, sig, frame):
        self.shut_down()
        sys.exit(0)

    def shut_down(self):
        self.move_to_zero()
        time.sleep(2)
        # self.move(self.terminate_angle)
        self.set_color(255,0,0)
        time.sleep(1)
        print("Shut down!")

    def update_info(self):
        self.joint_angle = self.robot.get_radians()
        self.coord = self.robot.get_coords()
        self.joint_speed = self.robot.get_speed()
        self.is_joint_moving = self.robot.is_moving()
        self.is_gripper_moving = self.robot.is_gripper_moving()
        if (self.is_joint_moving):
            self.set_color(255,255,0)
        time.sleep(0.001)




# if __name__ == "__main__":
#     real_robot_env = RealTaskBase()

#     action_scale = 0.2
#     angle = real_robot_env.get_init_state()
#     while True:
#         rand_action = action_scale*(np.zeros(6))
#         rand_action[2] = -action_scale
#         next_angle = real_robot_env.step(rand_action)

#         angle = real_robot_env.get_angle()
#         coord = real_robot_env.get_coord()
#         speed = real_robot_env.get_speed()
#         is_joint_moving = real_robot_env.get_is_joint_moving()
#         is_gripper_moving = real_robot_env.get_is_gripper_moving()
        
#         angle = next_angle
#         img = real_robot_env.get_rgb_image()
#         cv2.imwrite("asd.png", img)
#         print(f"\nangle: {angle}\ncoord: {coord}\nspeed: {speed}\nis_joint_moving: {is_joint_moving}\nis_gripper_moving: {is_gripper_moving}")
#         # print(f"angle: {angle}")#\nspeed: {speed}\nis_joint_moving: {is_joint_moving}\nis_gripper_moving: {is_gripper_moving}")

if __name__ == "__main__":
    real_robot_env = RealTaskBase()
    from scipy.spatial.transform import Rotation as R

    action_scale = 0.2
    curr_angle = real_robot_env.get_init_state()
    while True:
        desired_pos = np.matrix(np.identity(4))
        position = np.matrix([-0.1, -0.2, 0.2]).T
        r = R.from_rotvec(np.pi * np.array([0, -1, 0]))
        
        desired_pos[:3, 3] = position
        desired_pos[:3, :3] = r.as_matrix()
        desired_angle = utils.invKine(desired_pos, curr_angle)
        # desired_angle[0] = 0
        rand_action = action_scale*(np.zeros(6))
        next_angle = real_robot_env.step(desired_angle, False)
        # next_angle = real_robot_env.step(rand_action)
        print(f"desired_pos: {desired_pos}")
        print(f"desired_angle: {desired_angle}" )
        print(f"next_angle: {next_angle}" )
        
        # rand_action[2] = -action_scale
        
        # desired_pos = np.identity(4)
        # position = np.array([0.2, 0.0, 0.2])
        # curr_angle = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

        

        # utils.invKine()
        curr_angle = real_robot_env.get_angle()
        coord = real_robot_env.get_coord()
        speed = real_robot_env.get_speed()
        is_joint_moving = real_robot_env.get_is_joint_moving()
        is_gripper_moving = real_robot_env.get_is_gripper_moving()
        
        curr_angle = next_angle
        # img = real_robot_env.get_rgb_image()
        # cv2.imwrite("asd.png", img)
        print(f"\nangle: {curr_angle}\ncoord: {coord}\nspeed: {speed}\nis_joint_moving: {is_joint_moving}\nis_gripper_moving: {is_gripper_moving}")
        # print(f"angle: {angle}")#\nspeed: {speed}\nis_joint_moving: {is_joint_moving}\nis_gripper_moving: {is_gripper_moving}")