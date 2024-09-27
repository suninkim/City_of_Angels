from .coffee_task import RealCoffeeTask
from .dancing_task import RealDancingTask
from task_base import *
from .manager_based import *

real_task_map = {"coffee": RealCoffeeTask, "dancing": RealDancingTask}
sim_task_map = {"coffee": RealCoffeeTask, "dancing": RealDancingTask}
