import time

import numpy as np
import yaml

from planner import TaskPlanner

# from planner import TaskPlanner
from tasks.task_base import RealTaskBase  # , SimTaskBase
from vision import VisionSystem


class RealDancingTask(RealTaskBase):
    def __init__(self, task_cfg):
        super().__init__()
        self.is_task_done = False
        self.task_cfg = task_cfg
        self.is_target_set = False
        self.cnt = 0

        self.vision_system = VisionSystem(task_cfg["vision"])
        self.task_planner = TaskPlanner(task_cfg["planner"])
        self.current_task = None

    def do_task(self):
        next_task = self.task_planner.get_next_task(self.get_rgb_image())

        if next_task != self.current_task:
            print(f"Task is change from {self.current_task} to {next_task}")
            self.current_task = next_task

    def get_target_pose(self, target):
        object_xyz = self.vision_system.get_object_position(target)
        joint_angle = self.calculate_joint_angle(object_xyz)
        return joint_angle

    def calculate_joint_angle(self, ee_xyz):
        inv_angle = np.array([0, 0, 0, 0, 0, 0])
        return inv_angle

    def do_task(self):
        self.is_task_done = self.find_object()

        if self.is_task_done:
            a = 1

        # self.go_to_water_pose()
        # self.grab_water_bottle()
        # self.pour_water_into_cup()

    def go_to_cup_pose(self):
        cup_pose = self.get_target_pose("cup")

        camera_position = np.array([0.001, -1.298, 2.003, 0.268, -1.344, 1.433])

        grab_position = np.array([1.055, 1.751, 0.897, -1.603, -3.04, 2.602])
        pour_position = np.array([-0.455, 0.549, 0.545, 0.54, -1.805, 2.534])
        pour_to_cup_position = np.array([-0.987, 0.471, 0.452, 0.789, -1.006, 2.531])

        a = 1

    def go_to_water_pose(self):
        water_bottle_pose = self.get_target_pose("water_bottle")
        water_position = np.array([1.41, 1.049, 1.828, -1.066, -3.036, 2.902])
        self.move(water_position)
        time.sleep(3)

    def homing(self):
        a = 1

    def find_object(self):
        if self.is_task_done:
            a = 1
            return
        camera_position = np.array([0.001, -1.298, 2.003, 0.268, -1.344, 1.433])
        if not self.is_target_set:
            print("befor target set!")
            self.update_info()
            self.curr_angle = self.get_angle()
            self.diff = camera_position - self.curr_angle
            self.is_target_set = True
        else:
            new_position = self.curr_angle + self.cnt * self.diff / 10
            self.cnt += 1
            self.move(new_position)
        return True if self.cnt == 11 else False

    def look_around(self):
        object_list = []
        return object_list

    def plan_trajectory(self):
        trajectory = []
        return trajectory

    def grab_the_spoon(self):
        move_successed = False

    def scoop_coffee(self):
        mylist = [True, True, True]
        x = all(mylist)
        print(x)

    def pour_coffee_into_cup(self):
        a = 1

    def grab_water_bottle(self):
        grab_position = np.array([1.055, 1.751, 0.897, -1.603, -3.04, 2.602])
        self.move(grab_position)
        time.sleep(3)
        self.gripper_control(False)
        time.sleep(1)

    def pour_water_into_cup(self):
        pour_position = np.array([-0.455, 0.549, 0.545, 0.54, -1.805, 2.534])
        pour_to_cup_position = np.array([-0.987, 0.471, 0.452, 0.789, -1.006, 2.531])
        self.move(pour_position)
        time.sleep(3)
        self.move(pour_to_cup_position)
        time.sleep(3)

    def mix_component(self):
        a = 1

    def serve_coffee(self):
        a = 1


if __name__ == "__main__":
    task_cfg = yaml.load("")
    coffee_task_env = RealCoffeeTask(task_cfg)
    while True:
        coffee_task_env.do_task()
