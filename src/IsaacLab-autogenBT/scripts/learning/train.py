# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to train RL agent with RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

from omni.isaac.lab.app import AppLauncher

# local imports
# import cli_args  # isort: skip


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
# parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
# parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
# parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--num_search_agents", type=int, default=64, help="Number of search agents.")
parser.add_argument("--num_search_times", type=int, default=200, help="Number of search agents.")
parser.add_argument("--num_eval_agents", type=int, default=10, help="Number of evaluation agents.")
parser.add_argument("--training_iters", type=int, default=100, help="Training iterations.")
parser.add_argument("--env_spacing", type=int, default=40, help="Space between environments.")
# parser.add_argument("--task", type=str, default=None, help="Name of the task.")
# parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
# parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
# append RSL-RL cli arguments
# cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# # always enable cameras to record video
# if args_cli.video:
#     args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import torch
from datetime import datetime

from rsl_rl.runners import OnPolicyRunner

from omni.isaac.lab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from omni.isaac.lab.utils.dict import print_dict
from omni.isaac.lab.utils.io import dump_pickle, dump_yaml
from omni.isaac.lab_tasks.utils import get_checkpoint_path
from omni.isaac.lab_tasks.utils.hydra import hydra_task_config
from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlVecEnvWrapper

# Import extensions to set up environment tasks
# import autogen_bt.tasks  # noqa: F401

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False

# =====================================================================================================
from torch.utils.data import Dataset, DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter
import numpy as np
import rclpy
import time

# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
simulation_dir = os.path.abspath(os.path.join(project_root, "scripts", "simulation"))
bt_dir = os.path.abspath(os.path.join(project_root, "scripts", "bt"))

from gymEnv import MultiBTEnv
from mcts import MCTS
from network import RvNN_mem, RvNN

class BTDataset(Dataset):
    def __init__(self, bt_strings, action1_probs, action2_probs, rewards, device):
        """
        :param bt_strings: List of behavior tree strings
        :param action1_probs: List of node type distributions (numpy or list)
        :param action2_probs: List of location distributions (numpy or list)
        """
        self.bt_strings = bt_strings
        self.action1_probs = action1_probs
        self.action2_probs = action2_probs
        self.rewards = rewards
        self.device = device

    def __len__(self):
        return len(self.bt_strings)

    def __getitem__(self, idx):
        # You can convert numpy arrays to torch.Tensor here if needed
        return (
            self.bt_strings[idx],
            torch.tensor(self.action1_probs[idx], dtype=torch.float32, device=self.device),
            torch.tensor(self.action2_probs[idx], dtype=torch.float32, device=self.device),
            torch.tensor(self.rewards[idx], dtype=torch.float32, device=self.device)
        )
    
def modify_bt(current_bt, node_type, node_location):
    """Modify the BT string"""
    # Interpret node_type
                    # Flow Control
    node_dict = {   0 : '(0)', #patrol_node
                    1 : '(1)', #find_target_node
                    2 : '(2)', #go_to_nearest_target
                    # Behaviors
                    3 : 'a', #patrol_node
                    4 : 'b', #find_target_node
                    5 : 'c', #go_to_nearest_target
                    6 : 'd', #go_to_charger_node
                    7 : 'e', #go_to_spawn_node
                    8 : 'f', #picking_object_node
                    9 : 'g', #drop_object_node
                    10 : 'h', #charge_node
                    # Conditions
                    11 : 'A', #is_robot_at_the_charger_node
                    12 : 'B', #is_robot_at_the_spawn_node
                13 : 'C', #is_battery_on_proper_level
                14 : 'D', #are_object_existed_on_internal_map
                15 : 'E', #are_object_nearby_node
                16 : 'F', #is_object_in_hand_node
                17 : 'G', #is_nearby_object_not_at_goal
                18 : 'H', #are_five_objects_at_spawn
                # Specials
                19 : None, #stop node
                }
    node = node_dict[node_type]

    # Add a node to the BT string at the specified location
    bt_string = current_bt

    if bt_string == '':
        # If the string is empty, just return the node
        if node is None:
            return ''
        return node

    if node != None:
        # Iterate over all potential insertion positions (0 to len(s))
        valid_indices = [j for j in range(1,len(bt_string)) if j == len(bt_string) or not bt_string[j].isdigit()]

        if 0 <=  node_location < len(valid_indices):
            # Get the actual index in the string where we want to insert the char.
            insert_index = valid_indices[node_location]

            # Return the new string with the character inserted.
            return bt_string[:insert_index] + node + bt_string[insert_index:]
        
    return current_bt

def dataset_generation(num_search_agents, num_search, eval_env, policy_net, num_node_to_explore = 10, device='cuda:0', verbose = False):
    policy_net = policy_net.to(device)

    env = MultiBTEnv(num_envs=num_search_agents)
    mcts = MCTS(env, policy_net, num_simulations=num_search, exploration_weight=1.0, device=device)

    bt_string = ''

    bt_strings = []
    action1_probs = []
    action2_probs = []

    number_of_nodes = 0
    max_nodes = 50

    while True:
        # Calculate the temperature
        if number_of_nodes < num_node_to_explore:
            temperature = 1.0
        else:
            temperature = 1/(number_of_nodes - (num_node_to_explore - 1))

        # Get the action probabilities from MCTS search
        nt_probs, loc_probs = mcts.run_search(root_state=bt_string,temperature=temperature, verbose=True)

        # Store the sample data
        bt_strings.append(bt_string)
        action1_probs.append(nt_probs)
        action2_probs.append(loc_probs)
        
        max_nt_probs = np.max(nt_probs)
        max_loc_probs = np.max(loc_probs)

        best_nt_indices = np.where(nt_probs == max_nt_probs)[0]
        best_loc_indices = np.where(loc_probs == max_loc_probs)[0]

        # Randomly select one of the best indices
        selected_nt = np.random.choice(best_nt_indices)
        selected_loc = np.random.choice(best_loc_indices)

        if verbose: print(f"[INFO] Dataset {number_of_nodes + 1}/{max_nodes} << {bt_strings[-1]}, ({selected_nt}, {selected_loc})")

        if selected_nt == 19 or number_of_nodes >= max_nodes:
            break

        bt_string = modify_bt(bt_string, selected_nt, selected_loc)
        number_of_nodes += 1

    for env_id in range(eval_env.num_envs):
        eval_env.set_bt(env_id=env_id, bt_string=bt_string)

    # Get the reward by runnung the BT in IsaacSim Simulation
    if verbose: print(f"[INFO] \tEvaluating {bt_string}")
    _, rews, _, infos =  eval_env.evaluate_bt_in_sim()
    rew = np.mean(rews)
    if verbose: print(f"[INFO] \tFinished Evaluation >> reward: {rew}")

    # Convert rewards to numpy array
    rewards = np.array([rew] * len(bt_strings))
    
    # Convert list of np.array to a single np.array
    action1_probs = np.array(action1_probs)
    action2_probs = np.array(action2_probs)

    return BTDataset(bt_strings, action1_probs, action2_probs, rewards, device=device)

if __name__ == "__main__":
    # rclpy.init()
    device = args_cli.device

    # Instantiate the model
    model = RvNN(
        node_type_vocab_size=20,  # 0 to 19
        embed_size=64,
        hidden_size=128,
        action1_size=20,          # Number of node types to choose from
        action2_size=99,          # Max insertion locations
        device=device
    )

    # Optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)

    eval_env = MultiBTEnv(num_envs=args_cli.num_eval_agents, simulation_app=simulation_app, env_spacing=args_cli.env_spacing, device=device)
    
    # TensorBoard writer setup
    log_dir = os.path.join(project_root, "logs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    writer = SummaryWriter(log_dir=log_dir)

    global_step = 0
    start_time = time.time()
    for iter_i in range(args_cli.training_iters):
        print(f"[INFO] Iteration {iter_i + 1}/{args_cli.training_iters}")

        # Generate dataset
        dataset = dataset_generation(
            num_search_agents=args_cli.num_search_agents,
            num_search=args_cli.num_search_times,
            eval_env=eval_env,
            policy_net=model,
            device=device,
            verbose=True
        )

        train_loader = DataLoader(dataset, batch_size=4, shuffle=True)

        # Train and log metrics
        avg_loss = model.train_loop(
            train_loader=train_loader,
            optimizer=optimizer,
            num_epochs=20,
            l2_weight=1e-4,
            writer=writer,
            global_step=global_step
        )

        # Log final evaluation reward
        writer.add_scalar("Eval/FinalReward", dataset.rewards[0].item(), iter_i)

        # Save model after each iteration (optional: adjust to save best only)
        model_path = os.path.join(log_dir, f"rvnn_iter{iter_i:03d}.pt")
        torch.save(model.state_dict(), model_path)
        print(f"[INFO] Model saved at {model_path}")

        global_step += 20  # assuming 20 epochs per iteration

    # Save final model
    final_model_path = os.path.join(log_dir, "rvnn_final.pt")
    torch.save(model.state_dict(), final_model_path)
    print(f"[INFO] Final model saved at {final_model_path}")

    # Close TensorBoard writer
    writer.close()

    print(f"Final Results: {dataset.bt_strings[-1]}")
    print(f"Total elapsed time: {time.time() - start_time:.2f} seconds")
    rclpy.try_shutdown()


# =====================================================================================================

# @hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
# def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
#     """Train with RSL-RL agent."""
#     # override configurations with non-hydra CLI arguments
#     agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
#     env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
#     agent_cfg.max_iterations = (
#         args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
#     )

#     # set the environment seed
#     # note: certain randomizations occur in the environment initialization so we set the seed here
#     env_cfg.seed = agent_cfg.seed
#     env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

#     # specify directory for logging experiments
#     log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
#     log_root_path = os.path.abspath(log_root_path)
#     print(f"[INFO] Logging experiment in directory: {log_root_path}")
#     # specify directory for logging runs: {time-stamp}_{run_name}
#     log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
#     if agent_cfg.run_name:
#         log_dir += f"_{agent_cfg.run_name}"
#     log_dir = os.path.join(log_root_path, log_dir)

#     # create isaac environment
#     env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
#     # wrap for video recording
#     if args_cli.video:
#         video_kwargs = {
#             "video_folder": os.path.join(log_dir, "videos", "train"),
#             "step_trigger": lambda step: step % args_cli.video_interval == 0,
#             "video_length": args_cli.video_length,
#             "disable_logger": True,
#         }
#         print("[INFO] Recording videos during training.")
#         print_dict(video_kwargs, nesting=4)
#         env = gym.wrappers.RecordVideo(env, **video_kwargs)

#     # convert to single-agent instance if required by the RL algorithm
#     if isinstance(env.unwrapped, DirectMARLEnv):
#         env = multi_agent_to_single_agent(env)

#     # wrap around environment for rsl-rl
#     env = RslRlVecEnvWrapper(env)

#     # create runner from rsl-rl
#     runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
#     # write git state to logs
#     runner.add_git_repo_to_log(__file__)
#     # save resume path before creating a new log_dir
#     if agent_cfg.resume:
#         # get path to previous checkpoint
#         resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
#         print(f"[INFO]: Loading model checkpoint from: {resume_path}")
#         # load previously trained model
#         runner.load(resume_path)

#     # dump the configuration into log-directory
#     dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
#     dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
#     dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
#     dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)

#     # run training
#     runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

#     # close the simulator
#     env.close()


# if __name__ == "__main__":
#     # run the main function
#     main()
#     # close sim app
#     simulation_app.close()
