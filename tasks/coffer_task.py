import yaml
import numpy as np
import time

# from planner import TaskPlanner
from task_base import RealTaskBase#, SimTaskBase

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

class RealCoffeeTask(RealTaskBase):

    def __init__(self, task_cfg):
        super().__init__()
        self.is_task_done = False
        self.task_cfg = task_cfg
        # self.task_planner = TaskPlanner()
        self.is_target_set = False
        self.cnt = 0

    def do_task(self):
        self.is_task_done = self.find_object()

        if self.is_task_done:
            a=1 

        # self.go_to_water_pose()
        # self.grab_water_bottle()
        # self.pour_water_into_cup()

    def go_to_cup_pose(self):
        camera_position = np.array([0.001, -1.298, 2.003, 0.268, -1.344, 1.433])
        
        grab_position = np.array([1.055, 1.751, 0.897, -1.603, -3.04, 2.602])
        pour_position = np.array([-0.455, 0.549, 0.545, 0.54, -1.805, 2.534])
        pour_to_cup_position = np.array([-0.987, 0.471, 0.452, 0.789, -1.006, 2.531])
        
        a=1

    def go_to_water_pose(self):
        water_position = np.array([1.41, 1.049, 1.828, -1.066, -3.036, 2.902])
        self.move(water_position)
        time.sleep(3)
        
    def homing(self):
        a=1
    
    def find_object(self):
        if self.is_task_done:
            a=1
            return
        camera_position = np.array([0.001, -1.298, 2.003, 0.268, -1.344, 1.433])
        if not self.is_target_set:
            print("befor target set!")
            self.update_info()
            self.curr_angle = self.get_angle()
            self.diff = camera_position - self.curr_angle
            self.is_target_set = True
        else:
            new_position = self.curr_angle + self.cnt * self.diff/10
            self.cnt+=1
            self.move(new_position)
        return True if self.cnt ==11 else False
    
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
        a=1

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
        a=1

    def serve_coffee(self):
        a=1

if __name__ == "__main__":
    task_cfg = yaml.load("")
    coffee_task_env = RealCoffeeTask(task_cfg)
    while True:
        coffee_task_env.do_task()