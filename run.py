import argparse
import os
import shutil
from datetime import datetime

import isaacgym
import yaml

from utils.run_utils import setup_logger_kwargs


def homing():
    a=1

def run(args):


    task_cfg_path = os.path.join('./runs', args.exp_name, 'task_cfg.yaml') if (args.test or args.resume or args.collect or  args.dynamics) and args.exp_name is not None \
                    else os.path.join('cfg', 'task', args.task_cfg + '.yaml')

    with open(task_cfg_path, 'r') as stream:
        task_cfg = yaml.safe_load(stream)

    train_cfg_path = os.path.join('./runs', args.exp_name, 'train_cfg.yaml') if (args.test or args.resume or args.collect or args.dynamics) and args.exp_name is not None \
                    else os.path.join('cfg', 'train', args.train_cfg + '.yaml')
    with open(train_cfg_path, 'r') as stream:
        train_cfg = yaml.safe_load(stream)

    if not (args.test or args.resume or args.dynamics) and args.date :
        args.exp_name = '{}-{}'.format( args.exp_name, datetime.now().strftime("%Y-%m-%d-%H-%M"))


    if not (args.test or args.resume or args.collect or args.dynamics):
        os.makedirs(os.path.join('./runs', args.exp_name), exist_ok = True)
        shutil.copy(task_cfg_path, os.path.join('./runs', args.exp_name, 'task_cfg.yaml'))
        shutil.copy(train_cfg_path, os.path.join('./runs', args.exp_name, 'train_cfg.yaml'))

    from algos import algos_map
    algo_fn = algos_map[train_cfg['algo']]
    algo = algo_fn(task_cfg, train_cfg, args)

    if args.test:
        if not args.vision: 
            algo.test()
        else:
            algo.vision_test()
    elif args.collect: 
        algo.collect()
    else:
        algo.train(args.resume)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--task_cfg', type=str, default='ambidex_ramen_taskspace')
    parser.add_argument('--train_cfg', type=str, default='ppo_ramen_taskspace')
    parser.add_argument('--sim', action='store_true')
    parser.add_argument('--real', action='store_true')
    args = parser.parse_args()

    run(args)
