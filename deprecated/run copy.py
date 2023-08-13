import argparse
import os
import shutil
import sys
from datetime import datetime

import yaml

from algos import *
from tasks import *
from utils.run_utils import setup_logger_kwargs


def run(args):
    task_cfg_path = (
        os.path.join("./runs", args.exp_name, "task_cfg.yaml")
        if (args.test or args.resume or args.collect or args.dynamics)
        and args.exp_name is not None
        else os.path.join("cfg", "task", args.task_cfg + ".yaml")
    )

    with open(task_cfg_path, "r") as stream:
        task_cfg = yaml.safe_load(stream)

    train_cfg_path = (
        os.path.join("./runs", args.exp_name, "train_cfg.yaml")
        if (args.test or args.resume or args.collect or args.dynamics)
        and args.exp_name is not None
        else os.path.join("cfg", "train", args.train_cfg + ".yaml")
    )
    with open(train_cfg_path, "r") as stream:
        train_cfg = yaml.safe_load(stream)

    if not (args.test or args.resume or args.dynamics) and args.date:
        args.exp_name = "{}-{}".format(
            args.exp_name, datetime.now().strftime("%Y-%m-%d-%H-%M")
        )

    if not (args.test or args.resume or args.collect or args.dynamics):
        os.makedirs(os.path.join("./runs", args.exp_name), exist_ok=True)
        shutil.copy(
            task_cfg_path, os.path.join("./runs", args.exp_name, "task_cfg.yaml")
        )
        shutil.copy(
            train_cfg_path, os.path.join("./runs", args.exp_name, "train_cfg.yaml")
        )

    # Set up the robot
    if args.sim:
        env = SimTaskBase()
    else:
        env = RealTaskBase()

    if args.task == "train":
        # Set up the training loop
        trainer = SAC()

        # Start training
        trainer.train(env)

        # Save the final checkpoint
        trainer.save_checkpoint(
            os.path.join("./runs", args.exp_name, "final_checkpoint.pt")
        )
    else:
        env.do_task()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--task_cfg", type=str, default="coffee")
    parser.add_argument("--train_cfg", type=str, default="sac_skill")
    parser.add_argument("--sim", action="store_true")
    args = parser.parse_args()
    run(args)
