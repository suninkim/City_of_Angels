import json
import math
import os
import threading
import time

import numpy as np
import pyrealsense2 as rs
import serial
import serial.tools.list_ports
from pymycobot import PI_BAUD, PI_PORT
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot

from utils import utils


class RealTaskBase():
    def __init__(self, port='/dev/ttyAMA0', baudrate="115200", timeout=0.1, debug=False):

        self.robot = MyCobot(port, baudrate, debug=debug)

        self.homing_angle = np.array([-0.2, 0.45, 0.85, 0.27, -1.57, 1.37]) # change to yaml
        self.terminate_angle = np.array([0.0, 1.67, 1.0, -1.1, 0.0, 0.0]) # change to yaml

        self.robot.set_gripper_mode(0)
        self.gripper_mode = self.robot.get_gripper_mode()

        self.is_calibrated = False # change to yaml

        if not self.is_calibrated:
            if str(input("Robot is not calibrated. Do you want to perform calibration? [Y/N]")) == "Y":
                self.calibration_robot()
            else:
                raise Exception("Cannot proceed without calibrating the robot first.")

        self.setup_camera()
        self.setup_thread()

        self.move(self.homing_angle)

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
            print("Calibration test finisihed!")

        
    ###########################
    #####  Basic Movement  ####
    ###########################

    def move(self, command, speed=20, coordinate="js"):

        is_motor_connected = self.check_servo()
        if coordinate == "js":
            self.robot.sync_send_angles(utils.rad2deg(command), speed, timeout=2)
        elif coordinate == "os":
            self.robot.sync_send_coords(utils.convert_os_command(command), speed, timeout=2)


    def gripper_control(self, open):
        if open:
            self.robot.set_gripper_value(100, 50)
        else:
            self.robot.set_gripper_value(20, 50)
        time.sleep(1)
        self.gripper_value = self.robot.get_gripper_value()

    def move_to_zero(self):
        self.move([0]*6)
        self.gripper_control(True)

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
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        return color_image

    def get_depth_image(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        depth_image = np.asanyarray(depth_frame.get_data())
        return depth_image
    
    def get_angle(self):
        return self.joint_angle

    def get_speed(self):
        return self.joint_speed
    
    def get_is_joint_moving(self):
        return self.is_joint_moving
    
    def get_is_gripper_moving(self):
        return self.is_gripper_moving

    ###########################
    ######   Connection  ######
    ###########################

    def setup_thread(self):

        # update robot informatiion
        self.joint_angle = None
        self.joint_angle_lock = threading.Lock() 
        self.joint_angle_thread = threading.Thread(target=self.update_angle, daemon=True)
        self.joint_angle_thread.start()

        self.joint_speed = None
        self.joint_speed_lock = threading.Lock() 
        self.joint_speed_thread = threading.Thread(target=self.update_speed, daemon=True)
        self.joint_speed_thread.start()

        self.is_joint_moving = False
        self.is_gripper_moving = False
        self.moving_lock = threading.Lock() 
        self.is_moving_thread = threading.Thread(target=self.update_is_moving, daemon=True)
        self.is_moving_thread.start()

    def update_angle(self):
        while True:
            new_angle = self.robot.get_radians()
            with self.joint_angle_lock:
                self.joint_angle = new_angle 
            time.sleep(0.01) 
    
    def update_speed(self):
        while True:
            new_speed = self.robot.get_speed()
            with self.joint_speed_lock:
                self.joint_speed = new_speed 
            time.sleep(0.01) 

    def update_is_moving(self):
        while True:
            is_joint_moving = self.robot.is_moving()
            is_gripper_moving = self.robot.is_gripper_moving()
            with self.moving_lock: 
                self.is_joint_moving = is_joint_moving 
                self.is_gripper_moving = is_gripper_moving 
            time.sleep(0.01) 

    def setup_camera(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(self.config)

    # def connect_mycobot(self):
    #     self.prot = port = self.port_list.get()
    #     if not port:
    #         print("Please select a serial port.")
    #         return
    #     self.baud = baud = self.baud_list.get()
    #     if not baud:
    #         print("Please select a baud rate.")
    #         return
    #     baud = int(baud)

    #     try:
    #         self.robot = MyCobot(port, baud)
    #         time.sleep(0.5)
    #         self.robot._write([255,255,3,22,1,250])
    #         time.sleep(0.5)
    #         print("Connected successfully!")
    #     except Exception as e:
    #         err_log = """\
    #             \rFailed to connect !!!
    #             \r=================================================
    #             {}
    #             \r=================================================
    #         """.format(
    #             e
    #         )

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

    def shut_down(self):
        self.move(self.terminate_angle)
        self.pipeline.stop() 
        print("Shut down!")



if __name__ == "__main__":
    real_robot_env = RealTaskBase()
    
    while True:
        angle = real_robot_env.get_angle()
        speed = real_robot_env.get_speed()
        is_joint_moving = real_robot_env.get_is_joint_moving()
        is_gripper_moving = real_robot_env.get_is_gripper_moving()
        print(f"angle: {angle}\nspeed: {speed}\nis_joint_moving: {is_joint_moving}\nis_gripper_moving: {is_gripper_moving}")