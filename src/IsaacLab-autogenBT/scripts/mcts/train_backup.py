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
import cli_args  # isort: skip


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument("--video_interval", type=int, default=2000, help="Interval between video recordings (in steps).")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--max_iterations", type=int, default=None, help="RL Policy training iterations.")
# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()

# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

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
import autogen_bt.tasks  # noqa: F401

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False

# =====================================================================================================
###
#  Simulation Configuration
###
import torch
import numpy as np
import sys
import os
import rclpy
import code 
from scipy.spatial.transform import Rotation
import time
import subprocess

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass

# Import local files
# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, ".."))

# Add it to sys.path 
sys.path.insert(0, project_root)
sys.path.insert(0, script_dir)

from assets.jackal_ur5 import JACKAL_UR5_CFG
from ros2_nodes.ros2_scene_publisher import pub_scene_data, SceneNode
from ros2_nodes.ros2_battery import BatteryManager
from ros2_nodes.ros2_object import ObjectGroupManager
from ros2_nodes.ros2_drive import RobotDriverManager

def get_random_object_pos(num_env):
    """
    Generate random target positions from a predefined pool.
    
    Args:
        num_env (int): Number of environments.
    
    Returns:
        torch.Tensor: Randomly selected target positions.
    """
    # Possible target spawn point
    target_spawn_pool = torch.tensor(
                    [[ 14.0, -4.0,  0.1],
                     [ 22.0, -1.0,  0.1],
                     [ 19.5,  7.7,  0.1],
                     [ 20.5, 13.0,  0.1],
                     [ 23.0, 21.0,  0.1],
                     [  9.0, 24.5,  0.1],
                     [ -2.0, 23.2,  0.1],
                     [ -1.5, 13.3,  0.1],
                     [ -2.6,  7.7,  0.1],
                     [  8.0,  6.7,  0.1]],device=args_cli.device)
    
    target_pos = torch.cat([target_spawn_pool[torch.randperm(len(target_spawn_pool))[:5]].unsqueeze(0) for _ in range(num_env)], dim=0)
    return target_pos

# Calculate initial rotation
map_rot = Rotation.from_euler('xyz', [90, 0, 0], degrees=True).as_quat()
robot_rot = Rotation.from_euler('xyz', [-630, -400, 60], degrees=True).as_quat()

@configclass
class ManualBTSceneCfg(InteractiveSceneCfg):
    """Configuration for a manual created BT robot scene."""

    # cave map
    map = AssetBaseCfg(prim_path='{ENV_REGEX_NS}/Map', 
        spawn=sim_utils.UsdFileCfg(usd_path=f"{project_root}/assets/cave_map.usd", scale=[0.0004, 0.0005, 0.0004]),
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-7, 27, 0], rot=[map_rot[3], map_rot[0], map_rot[1], map_rot[2]])
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # articulation
    robot: ArticulationCfg = JACKAL_UR5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

###
#   Behavior Tree Runner
###

processes = []

def run_BTs(bt_string_array, verbose=False):
    """
    Launch BTs as separate subprocesses.
    
    Args:
        bt_string_array (list): List of behavior tree strings.
    
    Returns:
        list: A list of subprocess.Popen objects representing the running BTs.
    """
    global processes
    processes = []  # Reset the global process list
    script_dir = os.path.dirname(os.path.abspath(__file__))
    run_bt_file = os.path.join(script_dir, "run_bt.py")

    for env_id, bt_string in enumerate(bt_string_array):
        # Run the run_bt.py script as a new process
        process = subprocess.Popen([
            'python3', run_bt_file,
            f'--env_id={env_id}',
            f"--bt_string={bt_string}"
        ])
        processes.append(process)
    
    return processes

def stop_BTs(verbose=False):
    """
    Stop all running BT processes by terminating them gracefully.
    """
    global processes
    for process in processes:
        if verbose: print(f"Terminating process with PID: {process.pid}")
        process.terminate()  # Send SIGTERM
    # Optionally, wait for each process to exit
    for process in processes:
        process.wait()
    if verbose: print("All BT processes have been terminated.")

###
#  Gym Environment
###

import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8

class BTEnv(gym.Env, Node):
    def __init__(self, env_id):
        super().__init__(f'bt_env_{env_id}')
        self.env_id = env_id
        
        # Action Space: (Node Type, Node Location)
        self.num_node_types = 20  # 20 possible node types
        self.action_space = gym.spaces.MultiDiscrete([self.num_node_types, 100])  # 100 is an arbitrary max tree size
        
        # Observation Space: String representing BT
        self.max_bt_length = 256  # Define a max length for encoding
        self.observation_space = gym.spaces.Text(max_length=self.max_bt_length)
        
        # ROS2 Communication - Reward Related
        self.object_at_spawn_subscriber = self.create_subscription(UInt8, f'/env_{env_id}/object_at_spawn', self.object_at_spawn_callback, 10)
        self.object_found_subscriber = self.create_subscription(UInt8, f'/env_{env_id}/object_found', self.object_found_callback, 10)      

        # Local Variables
        self.current_bt = ''
        self.object_at_spawn = 0
        self.object_found = 0
        self.current_reward = 0.0

    def object_at_spawn_callback(self, msg):
        """Receive reward from ROS2"""
        self.object_at_spawn = msg.data

    def object_found_callback(self, msg):
        """Receive reward from ROS2"""
        self.object_found = msg.data

    def step(self, action):
        """Modify the BT based on action"""
        node_type, node_location = action
        self.modify_bt(node_type, node_location)

        # Run Simulation
        

        # Publish new BT to ROS2
        msg = String()
        msg.data = self.current_bt
        self.bt_publisher.publish(msg)
        
        # Allow some time for execution
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # Update observation
        observation = self.current_bt
        reward = self.current_reward
        done = self.check_termination()
        
        return observation, reward, done, {}

    def reset(self):
        """Reset the environment"""
        self.current_bt = ''
        return self.current_bt

    def modify_bt(self, node_type, node_location):
        """Modify the BT string"""
        # Interpret node_type
                    # Flow Control
        node_dict = { 0 : '(0)', # sequence_node
                      1 : '(1)', # selector_node
                      2 : '(2)', # parallel_node
                    # Behaviors
                      3 : 'a',   # patrol_node
                      4 : 'b',   # find_target_node
                      5 : 'c',   # go_to_nearest_target
                      6 : 'd',   # go_to_charger_node
                      7 : 'e',   # go_to_spawn_node
                      8 : 'f',   # picking_object_node
                      9 : 'g',   # drop_object_node
                     10 : 'h',   # charge_node
                    # Conditions
                     11 : 'A',   # is_robot_at_the_charger_node
                     12 : 'B',   # is_robot_at_the_spawn_node
                     13 : 'C',   # is_battery_on_proper_level
                     14 : 'D',   # are_object_existed_on_internal_map
                     15 : 'E',   # are_object_nearby_node
                     16 : 'F',   # is_object_in_hand_node
                     17 : 'G',   # is_nearby_object_not_at_goal
                     18 : 'H',   # are_five_objects_at_spawn
                    # Specials
                     19 : None, #stop node
                    }
        node = node_dict[node_type]

        # Add a node to the BT string at the specified location
        bt_string = self.current_bt[1:-1]  # Remove the outer parentheses

        if node != None:
            valid_indices = []
        
            # Iterate over all potential insertion positions (0 to len(s))
            for i in range(len(bt_string) + 1):
                # If we are not at the end and the character at i is a digit, skip this index.
                if i < len(bt_string) and bt_string[i].isdigit():
                    continue
                valid_indices.append(i)
            
            if 0 <=  node_location < len(valid_indices):
                # Get the actual index in the string where we want to insert the char.
                insert_index = valid_indices[node_location]
        
                # Return the new string with the character inserted.
                self.current_bt = '(' + bt_string[:insert_index] + node + bt_string[insert_index:] + ')'

    def check_termination(self):
        """Check if the tree has reached an end state"""
        return len(self.current_bt) >= self.max_bt_length  # Example condition

    def initialize_bt(self):
        """Initialize a basic BT"""
        return "root"
    
###
#  Monte-Carlo Tree Search
###
import numpy as np
import random
import math

class MCTSNode:
    def __init__(self, state, parent=None, action=None):
        self.state = state  # BT as a string
        self.parent = parent
        self.action = action  # (Node Type, Node Location)
        self.children = []
        self.visits = 0
        self.value = 0.0

    def is_fully_expanded(self, action_space):
        """Check if all possible actions have been expanded"""
        existing_actions = {child.action for child in self.children}
        return len(existing_actions) >= action_space.nvec[0] * action_space.nvec[1]  # Total possible actions

    def best_child(self, exploration_weight=1.0):
        """Select the child with the best UCT score"""
        return max(
            self.children,
            key=lambda child: child.value / (child.visits + 1e-6) +
                              exploration_weight * math.sqrt(math.log(self.visits + 1) / (child.visits + 1e-6))
        )

class MCTS:
    def __init__(self, env, num_simulations=100):
        self.env = env
        self.num_simulations = num_simulations

    def select(self, node):
        """Select the most promising node using UCT"""
        while node.children:
            node = node.best_child()
        return node

    def expand(self, node):
        """Expand a new child node"""
        untried_actions = [(nt, nl) for nt in range(self.env.action_space.nvec[0])
                           for nl in range(self.env.action_space.nvec[1])]
        explored_actions = {child.action for child in node.children}
        possible_actions = [a for a in untried_actions if a not in explored_actions]

        if possible_actions:
            action = random.choice(possible_actions)
            new_state = self.env.modify_bt(*action)  # Modify BT with the action
            new_node = MCTSNode(new_state, parent=node, action=action)
            node.children.append(new_node)
            return new_node
        return node  # No expansion possible

    def simulate(self, node):
        """Simulate the rollout using a random policy"""
        rollout_env = self.env
        rollout_env.current_bt = node.state

        done = False
        total_reward = 0

        while not done:
            random_action = (random.randint(0, rollout_env.num_node_types - 1),
                             random.randint(0, 99))  # Example random action
            _, reward, done, _ = rollout_env.step(random_action)
            total_reward += reward

        return total_reward

    def backpropagate(self, node, reward):
        """Propagate the result back up the tree"""
        while node:
            node.visits += 1
            node.value += reward
            node = node.parent

    def search(self, initial_state):
        """Perform MCTS search"""
        root = MCTSNode(initial_state)

        for _ in range(self.num_simulations):
            node = self.select(root)
            node = self.expand(node)
            reward = self.simulate(node)
            self.backpropagate(node, reward)

        return root.best_child(exploration_weight=0).action  # Return best action (no exploration)

###
#  Neural Network for Policy and Value Estimation
###
import torch
import torch.nn as nn
import torch.optim as optim
from torch.nn.utils.rnn import pad_sequence

# class TreeRNN(nn.Module):
#     def __init__(self, vocab_size, embedding_dim, hidden_dim, action_dim):
#         super(TreeRNN, self).__init__()
#         self.embedding = nn.Embedding(vocab_size, embedding_dim)
#         self.rnn = nn.GRU(embedding_dim, hidden_dim, batch_first=True)
#         self.policy_head = nn.Linear(hidden_dim, action_dim)  # Output logits for policy
#         self.value_head = nn.Linear(hidden_dim, 1)  # Output single value estimate

#     def forward(self, bt_sequences):
#         """Forward pass for a batch of BTs"""
#         embedded = self.embedding(bt_sequences)  # Convert to embeddings
#         _, hidden = self.rnn(embedded)  # Pass through GRU
#         hidden = hidden.squeeze(0)  # Remove extra dimension

#         policy_logits = self.policy_head(hidden)  # Action probabilities
#         value = self.value_head(hidden)  # Value estimate

#         return policy_logits, value.squeeze(-1)
class TreeRNN(nn.Module):
    def __init__(self, vocab_size, embedding_dim, hidden_dim, num_node_types, num_locations):
        super(TreeRNN, self).__init__()
        self.embedding = nn.Embedding(vocab_size, embedding_dim)
        self.rnn = nn.GRU(embedding_dim, hidden_dim, batch_first=True)

        # Two separate policy heads
        self.policy_head_node_type = nn.Linear(hidden_dim, num_node_types)
        self.policy_head_location = nn.Linear(hidden_dim, num_locations)
        
        # Value head (optional for actor-critic or MCTS value guidance)
        self.value_head = nn.Linear(hidden_dim, 1)

    def forward(self, bt_sequences):
        embedded = self.embedding(bt_sequences)
        _, hidden = self.rnn(embedded)
        hidden = hidden.squeeze(0)

        logits_node_type = self.policy_head_node_type(hidden)
        logits_location = self.policy_head_location(hidden)
        value = self.value_head(hidden)

        return logits_node_type, logits_location, value.squeeze(-1)

# Example Usage:
if __name__ == "__main__":
    vocab_size = 50  # Number of possible BT node types
    embedding_dim = 32
    hidden_dim = 64
    action_dim = 19 * 100  # 19 node types * 100 possible positions

    model = TreeRNN(vocab_size, embedding_dim, hidden_dim, action_dim)
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # Example input (batch of BTs represented as padded sequences)
    example_bt = [torch.tensor([1, 2, 3, 0]), torch.tensor([4, 5, 0])]  # Padded input
    example_bt = pad_sequence(example_bt, batch_first=True, padding_value=0)
    
    policy, value = model(example_bt)
    print("Policy shape:", policy.shape)  # Should output (batch_size, action_dim)
    print("Value shape:", value.shape)  # Should output (batch_size,)

# =====================================================================================================

@hydra_task_config(args_cli.task, "rsl_rl_cfg_entry_point")
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlOnPolicyRunnerCfg):
    """Train with RSL-RL agent."""
    # override configurations with non-hydra CLI arguments
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs
    agent_cfg.max_iterations = (
        args_cli.max_iterations if args_cli.max_iterations is not None else agent_cfg.max_iterations
    )

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Logging experiment in directory: {log_root_path}")
    # specify directory for logging runs: {time-stamp}_{run_name}
    log_dir = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        log_dir += f"_{agent_cfg.run_name}"
    log_dir = os.path.join(log_root_path, log_dir)

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)
    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "train"),
            "step_trigger": lambda step: step % args_cli.video_interval == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)

    # create runner from rsl-rl
    runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=log_dir, device=agent_cfg.device)
    # write git state to logs
    runner.add_git_repo_to_log(__file__)
    # save resume path before creating a new log_dir
    if agent_cfg.resume:
        # get path to previous checkpoint
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)
        print(f"[INFO]: Loading model checkpoint from: {resume_path}")
        # load previously trained model
        runner.load(resume_path)

    # dump the configuration into log-directory
    dump_yaml(os.path.join(log_dir, "params", "env.yaml"), env_cfg)
    dump_yaml(os.path.join(log_dir, "params", "agent.yaml"), agent_cfg)
    dump_pickle(os.path.join(log_dir, "params", "env.pkl"), env_cfg)
    dump_pickle(os.path.join(log_dir, "params", "agent.pkl"), agent_cfg)

    # run training
    runner.learn(num_learning_iterations=agent_cfg.max_iterations, init_at_random_ep_len=True)

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
