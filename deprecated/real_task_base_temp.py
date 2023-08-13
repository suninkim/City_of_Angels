import json
import math
import os
import threading
import time
import numpy as np

import serial
import serial.tools.list_ports
from pymycobot import PI_BAUD, PI_PORT
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot

PI = math.pi
def rad2deg(rad):
    return rad * 180.0 / PI

# MyCobot function list
# get_radians()
# send_radians(radians, speed)
# sync_send_angles(degrees, speed, timeout=7)
# sync_send_coords(coords, speed, mode, timeout=7)
# gpio_init() 
# gpio_output(pin, v)
# release_all_servos()
# power_on()
# wait(t)

# TODO:
# homing test (include gripper) 
# configurate sim environment

class RealTaskBase():
    def __init__(self, port='/dev/ttyAMA0', baudrate="115200", timeout=0.1, debug=False):

        self.robot = MyCobot(port, baudrate, debug=debug)

        # self.homing_angle = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.homing_angle = np.array([-0.2, 0.45, 0.85, 0.27, -1.57, 1.37])
        self.terminate_angle = np.array([0.0, 1.67, 1.0, -1.1, 0.0, 0.0])

        self.robot.set_gripper_mode(0)
        self.gripper_mode = self.robot.get_gripper_mode()

    ###########################
    ######    Homing     ######
    ###########################

    def check_servo(self):
        res = []
        for i in range(1,8):
            _data = self.robot.get_servo_data(i , 5)
            time.sleep(0.02)
            # self.write_log_to_Text("connect servo error".format(_data))
            if _data != i:
                res.append(i)
        if res:
            print("Motor {} is not connected!".format(res))
            return False
        else:
            return True

    def homing(self):

        is_motor_connected = self.check_servo()

        self.homing_offset = 0* np.array([PI]*6)
        self.is_homing_done = False
        self.is_touch_limit = False
        self.robot.set_fresh_mode(0)
        
        print("Start homing.")
        self._homing()
        # self.homing_t = threading.Thread(target=_homing, daemon=True)
        # self.homing_t.start()
        # if self.is_homing_done:
        #     self.homing_t.join()

    def gripper_control(self, open):
        if open:
            self.robot.set_gripper_value(100, 50)
        else:
            self.robot.set_gripper_value(20, 50)
        time.sleep(1)

        self.gripper_value = self.robot.get_gripper_value()
        print(f"gripper value: {self.gripper_value}")

    def go_to_home_position(self):
        self.robot.sync_send_angles(rad2deg(np.zeros(6)), 30, timeout=3)
        self.gripper_control(True)

        self.robot.sync_send_angles(rad2deg(self.homing_angle), 30, timeout=2)
        self.gripper_control(False)

    def go_to_water_position(self):

        camera_position = np.array([0.001, -1.298, 2.003, 0.268, -1.344, 1.433])
        water_position = np.array([1.41, 1.049, 1.828, -1.066, -3.036, 2.902])
        grab_position = np.array([1.055, 1.751, 0.897, -1.603, -3.04, 2.602])
        pour_position = np.array([-0.455, 0.549, 0.545, 0.54, -1.805, 2.534])
        pour_to_cup_position = np.array([-0.987, 0.471, 0.452, 0.789, -1.006, 2.531])


        self.robot.sync_send_angles(rad2deg(camera_position), 30, timeout=5)
        self.robot.sync_send_angles(rad2deg(water_position), 30, timeout=5)
        self.gripper_control(True)
        self.robot.sync_send_angles(rad2deg(grab_position), 30, timeout=5)
        self.gripper_control(False)
        self.robot.sync_send_angles(rad2deg(pour_position), 30, timeout=5)
        self.robot.sync_send_angles(rad2deg(pour_to_cup_position), 30, timeout=5)


    def _homing(self):
        start_t = time.time()
        prev_angle = [PI] *6
        speed = 20
        cnt = 0
        print(f"self.robot.is_moving(): {self.robot.is_moving()}")
        while not self.is_touch_limit:
            print(f"cnt: {cnt}, self.robot.is_moving(): {self.robot.is_moving()}")
            angles = self.robot.get_radians()
            curr_time = time.time()
            print(f"time: {curr_time}, angles: {angles}")
            angle_command = np.array(angles).copy()
            angle_command[0] -= 0.5
            if angles[0] == prev_angle[0] and cnt > 3 :
                print("homing done")
                self.homing_offset[0] = angles[0]
                self.is_touch_limit = True
                angle_command[0] = angles[0]
            # self.robot.send_radians(list(angle_command), speed)
            time.sleep(0.2)
            prev_angle[0] = angles[0]
            cnt += 1
            # print(angles)
            # angles_command = np.array(angles)+ 0.1
            # speed = 10
            # if angles:
            #     for i in range(6):
            #         if angles[i] == prev_angle[i]:
            #             angles_command[i] = angles[i]
            #             self.homing_offset[i] = angles[i]
            #     if angles == prev_angle:
            #         self.is_touch_limit = True
            #     print(f'angles_command: {angles_command}')
            #     self.robot.send_radians(list(angles_command), speed)
            #     time.sleep(0.5)
            #     prev_angle[i] = angles[i]
        print("Go Home Position")
        print(f"homing angle: {self.homing_angle}")
        print(f"homing_offset: {self.homing_offset}")
        self.robot.sync_send_angles(rad2deg(self.homing_angle-self.homing_offset), speed, timeout=1)
        self.is_homing_done = True


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
    ######   Shut down   ######
    ###########################
    def shut_down(self):
        print("Shut down!")
        speed = 30
        self.robot.sync_send_angles(rad2deg(self.terminate_angle-self.homing_offset), speed, timeout=1)
    
    ###########################
    ###### Image process ######
    ###########################
    # TODO(SI): need camera to get
    def get_camera_image(self):
        a=1


    def check_mycobot_servos(self):
        self.connect_mycobot()
        if not self.has_mycobot():
            return

        res = []
        for i in range(1,8):
            _data = self.mycobot.get_servo_data(i , 5)
            time.sleep(0.02)
            # self.write_log_to_Text("connect servo error".format(_data))
            if _data != i:
                res.append(i)
        if res:
            self.write_log_to_Text("Joint(s) {} cannot communicate!!!".format(res))
        else:
            self.write_log_to_Text("All joints are connected normally.")

    def calibration_mycobot(self):
        """Calibration button click event.

        Click to calibrate one motor at a time and calibrate in turn. After all
        calibration, resume initialization.
        """
        if not self.has_mycobot():
            return

        if not self.calibration_num:
            self.calibration_num = 0

        self.calibration_num += 1

        self.mycobot.set_servo_calibration(self.calibration_num)
        time.sleep(0.1)
        self.mycobot.focus_servo(self.calibration_num)
        time.sleep(0.5)
        pos=self.mycobot.get_angles()
        self.write_log_to_Text("Calibration of motor "+str(self.calibration_num) + " completed.")

        if self.calibration_num == 6:
            self.write_log_to_Text("All calibration completed.")
            self.calibration_num = None
            # self.rectify_mycobot()
            self._calibration_test()

    def has_mycobot(self):
        """Check whether it is connected on mycobot"""
        if not self.mycobot:
            self.write_log_to_Text("mycobot has not been connected yet!!!")
            return False
        return True

    def connect_mycobot(self):
        self.prot = port = self.port_list.get()
        if not port:
            self.write_log_to_Text("Please select a serial port.")
            return
        self.baud = baud = self.baud_list.get()
        if not baud:
            self.write_log_to_Text("Please select a baud rate.")
            return
        baud = int(baud)

        try:
            # self.mycobot = MyCobot(PI_PORT, PI_BAUD)
            self.mycobot = MyCobot(port, baud)
            time.sleep(0.5)
            self.mycobot._write([255,255,3,22,1,250])
            time.sleep(0.5)
            # self.mycobot = MyCobot("/dev/cu.usbserial-0213245D", 115200)
            self.write_log_to_Text("Connected successfully!")
        except Exception as e:
            err_log = """\
                \rFailed to connect !!!
                \r=================================================
                {}
                \r=================================================
            """.format(
                e
            )
            self.write_log_to_Text(err_log)

    def write_log_to_Text(self, logmsg: str):
        global LOG_NUM
        current_time = self.get_current_time()
        logmsg_in = str(current_time) + " " + str(logmsg) + "\n"  # add a newline character



if __name__ == "__main__":
    real_robot_env = RealTaskBase()
    real_robot_env.go_to_home_position()
    # real_robot_env.go_to_water_position()

    # angles = real_robot_env.robot.get_radians()
    # print(f"Robot angle: {angles}")

    while real_robot_env.robot.is_moving():
        time.sleep(0.5)
    # real_robot_env.shut_down()