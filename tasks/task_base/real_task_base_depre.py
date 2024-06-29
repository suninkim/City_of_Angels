import atexit
import json
import math
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import serial
import serial.tools.list_ports
from pymycobot import PI_BAUD, PI_PORT
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))

from vision import arducam

PI = math.pi

def rad2deg(rad):
    return rad * 180.0 / PI

class RealTaskBase:
    def __init__(
        self, port="/dev/ttyAMA0", baudrate="115200", timeout=0.1, debug=False
    ):
        self.robot = MyCobot(port, baudrate, debug=debug)

        self.homing_angle = np.array(
            [-0.2, 0.45, 0.85, 0.27, -1.47, 1.37]
        )  # change to yaml
        self.terminate_angle = np.array(
            [0.0, 1.67, 1.0, -1.1, 0.0, 0.0]
        )  # change to yaml

        self.robot.set_gripper_mode(0)
        self.gripper_mode = self.robot.get_gripper_mode()

        self.is_calibrated = False  # change to yaml
        self.calibration_num = False

        self.setup_camera()
            
        if not self.is_calibrated:
            need_calibration = str(
                input(
                    "Robot is not calibrated. Do you want to perform calibration? [Y/N]"
                )
            )
            if need_calibration == "Y" or need_calibration == "y":
                for _ in range(6):
                    self.calibration_robot()
                    time.sleep(1)

        signal.signal(signal.SIGINT, self.signal_handler)
        atexit.register(self.shut_down)

        self.set_color(0, 255, 255)
        self.move(self.homing_angle)
        self.gripper_control(True)
        time.sleep(3)
        self.update_info()
        self.set_color(0, 255, 255)
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
            print("Failed to connect cobot!")
            return

        if not self.calibration_num:
            self.calibration_num = 0
        self.calibration_num += 1

        self.robot.set_servo_calibration(self.calibration_num)
        time.sleep(0.1)
        self.robot.focus_servo(self.calibration_num)
        time.sleep(0.5)
        pos = self.robot.get_angles()
        print("Calibration of motor " + str(self.calibration_num) + " completed.")

        if self.calibration_num == 6:
            print("Calibration is succeessed!")
            self.calibration_num = None
            self.calibration_test()

    def calibration_test(self):
        need_calibration_test = str(
                input(
                    "Robot is calibrated. Do you want to perform calibration test? [Y/N]"
                )
            )
        if need_calibration_test == "Y" or need_calibration_test == "y":
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
        else:
            return

    ###########################
    #####  Basic Movement  ####
    ###########################

    def move(self, command, speed=30, coordinate="js"):
        self.update_info()

        # is_motor_connected = self.check_servo()
        if coordinate == "js":
            self.robot.sync_send_angles(rad2deg(command), speed, timeout=0.001)
        elif coordinate == "os":
            self.robot.sync_send_coords(command, speed=speed, mode=0, timeout=2)

    def gripper_control(self, open):
        if open:
            self.robot.set_gripper_value(100, 50)
        else:
            self.robot.set_gripper_value(20, 50)
        time.sleep(1)
        self.gripper_value = self.robot.get_gripper_value()

    def move_to_zero(self):
        self.move(np.array([0, 0, 0, 0, 0, 0]))
        self.gripper_control(False)

    def step(self, action, relative=True, coordinate="js"):
        self.update_info()
        if relative:
            action_command = self.joint_angle.copy() + action
        else:
            action_command = action
        if coordinate == "js":
            self.move(action_command)
        else:
            self.move(action_command, coordinate="os")
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
        frame = self.arducam.get_frame()
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
        self.arducam = arducam.Arducam(video_num=0, publish_ros = True)
        time.sleep(1)
        if not self.arducam .get_publishing():
            self.arducam .pub_start()

    def set_color(self, r, g, b):
        self.robot.set_color(r, g, b)

    def has_mycobot(self):
        """Check whether it is connected on mycobot"""
        if not self.robot:
            print("mycobot has not been connected yet!!!")
            return False
        return True

    def check_servo(self):
        res = []
        for i in range(1, 8):
            _data = self.robot.get_servo_data(i, 5)
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
        self.set_color(255, 0, 0)
        time.sleep(1)
        print("Shut down!")

    def update_info(self):
        self.joint_angle = self.robot.get_radians()
        self.coord = self.robot.get_coords()
        self.joint_speed = self.robot.get_speed()
        self.is_joint_moving = self.robot.is_moving()
        self.is_gripper_moving = self.robot.is_gripper_moving()
        if self.is_joint_moving:
            self.set_color(255, 255, 0)
        time.sleep(0.001)


if __name__ == "__main__":
    real_robot_env = RealTaskBase()
    from scipy.spatial.transform import Rotation as R

    action_scale = 0.2
    curr_angle = real_robot_env.get_init_state()
    des_action = np.array([-250, 0, 300, 0, 175, 180])
    while True:
        rand_action = action_scale * (np.zeros(6))
        # next_angle = real_robot_env.step(rand_action, False)
        next_angle = real_robot_env.step(des_action, False, "os")
        print(f"next_angle: {next_angle}")

        curr_angle = real_robot_env.get_angle()
        coord = real_robot_env.get_coord()
        speed = real_robot_env.get_speed()
        is_joint_moving = real_robot_env.get_is_joint_moving()
        is_gripper_moving = real_robot_env.get_is_gripper_moving()

        curr_angle = next_angle
        print(
            f"\nangle: {curr_angle}\ncoord: {coord}\nspeed: {speed}\nis_joint_moving: {is_joint_moving}\nis_gripper_moving: {is_gripper_moving}"
        )
