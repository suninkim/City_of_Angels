import argparse
import os
import shutil
import sys
from datetime import datetime

import yaml

from algos import *
from tasks import *


def run(args):
    # Set up the robot
    task_cfg_path = f"cfg/{args.task_cfg}_task.yaml"

    with open(task_cfg_path, "r") as stream:
        task_cfg = yaml.safe_load(stream)

    if args.sim:
        env = sim_task_map[task_cfg["task"]["env"]](task_cfg["task"])
    else:
        env = real_task_map[task_cfg["task"]["env"]](task_cfg["task"])

    while True:
        env.do_task() 


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--task_cfg", type=str, default="coffee")
    parser.add_argument("--train_cfg", type=str, default="sac_skill")
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()
    run(args)
