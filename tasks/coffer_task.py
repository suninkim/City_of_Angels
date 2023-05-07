import yaml

from . import RealTaskBase

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

    def do_task(self):
        a=1

    def go_to_cup_pose(self):
        a=1

    def go_to_water_pose(self):
        g=1


if __name__ == "__main__":
    task_cfg = yaml.load("")
    coffee_task_env = RealCoffeeTask(task_cfg)
    while True:
        coffee_task_env.do_task()