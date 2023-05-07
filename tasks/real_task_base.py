import json
import math
import os
import threading
import time

import serial
import serial.tools.list_ports
from pymycobot import PI_BAUD, PI_PORT
from pymycobot.genre import Angle, Coord
from pymycobot.mycobot import MyCobot

PI = math.pi

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
    def __init__(self, port, baudrate="115200", timeout=0.1, debug=False):

        self.robot = MyCobot(port, baud=baudrate, debug=debug)

        self.robot.set_gripper_mode(0)
        self.gripper_mode = self.robot.get_gripper_mode()

        self.homing_angle = [PI]*6

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

        self.homing_offset = [PI]*6
        self.is_homing_done = False
        self.is_touch_limit = False
        self.robot.set_frech_mode(0)
        def _homing():
            start_t = time.time()
            prev_angle = [PI] *6
            while not self.is_touch_limit:
                angles = self.robot.get_encoders()
                angles_command = angles - 10
                speed = 10
                if angles:
                    for i in range(6):
                        if angles[i] == prev_angle[i]:
                            angles_command[i] = angles[i]
                            self.homing_offset[i] = angles[i]
                    if angles == prev_angle:
                        self.is_touch_limit = True
                    self.robot.sync_send_angles(angles_command, speed, timeout=1)
                    prev_angle[i] = angles[i]

            self.robot.sync_send_angles(self.homing_angle-self.homing_offset, speed, timeout=1)
            self.is_homing_done = True

        print("Start homing.")
        self.homing_t = threading.Thread(target=_homing, daemon=True)
        self.homing_t.start()
        if self.is_homing_done:
            self.homing_t.join()

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

    def gripper_control(self, open):
        if open:
            self.robot.set_gripper_value(100, 50)
        else:
            self.robot.set_gripper_value(30, 50)
        time.sleep(1)

        self.gripper_value = self.robot.get_gripper_value()

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
    real_robot_env.homing()