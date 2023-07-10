import yaml

from planner import TaskPlanner
from tasks.task_base import RealTaskBase, SimTaskBase

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
        super(RealTaskBase, self).__init__()
        self.task_cfg = task_cfg
        self.task_planner = TaskPlanner()

    def do_task(self):
        a=1

    def go_to_cup_pose(self):
        a=1

    def go_to_water_pose(self):
        g=1

    def homing(self):
        a=1
    
    def find_object(self, target):
        position = [1,2,3]
        return position
    
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

    def grib_water_bottle(self):
        a=1

    def pour_water_into_cup(self):
        a=1

    def mix_component(self):
        a=1

    def serve_coffee(self):
        a=1

if __name__ == "__main__":
    task_cfg = yaml.load("")
    coffee_task_env = RealCoffeeTask(task_cfg)
    while True:
        coffee_task_env.do_task()