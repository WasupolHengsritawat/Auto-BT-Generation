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
from ros2_nodes.ros2_bt_tracker import BTStatusTrackerNode

def get_random_object_pos(num_env, mode = 'different'):
    """
    Generate random object target positions from a predefined spawn pool.

    :param num_env: Number of parallel environments.
    :param mode: 'different' assigns a unique set of positions to each environment,
                 'same' assigns the same set to all.
    :return: A tensor of shape (num_env, 5, 3) containing randomly selected target positions.
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
    
    if mode == 'different': target_pos = torch.cat([target_spawn_pool[torch.randperm(len(target_spawn_pool))[:5]].unsqueeze(0) for _ in range(num_env)], dim=0)
    elif mode == 'same'   : target_pos = torch.cat([target_spawn_pool[torch.randperm(len(target_spawn_pool))[:5]].unsqueeze(0)] * num_env, dim=0)
    
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

import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, UInt8

class MultiBTEnv(gym.Env):
    def __init__(self, num_envs):
        """
        Initialize multiple BT environments and connect to ROS2 simulation.

        :param num_envs: Number of parallel environments (agents).
        """
        super().__init__()
        self.node = Node('multi_bt_env')
        self.num_envs = num_envs

        self.object_pos_mode = 'same' # 'same' if want all environment object positions to be the same
                                      # 'different' for otherwise
        self.idle_step_limit = 200
        self.sim_step_limit = 200000
        self.nodes_limit = 50
        self.number_of_target_to_success = 5

        self.reward_weight = [ 100,  # Number of delivered objects term
                                70,  # Number of founded objects term
                               100,  # Bonus time reward if success term
                              -250,  # Battery dead penalty term 
                                -1]  # Tree complexity penalty term

        self._create_sim()  # Create and initialize the simulation

        # Action Space: (Node Type, Node Location)
        self.num_node_types = 20  # 20 possible node types
        self.max_location_size = 50  # 50 maximum location size
        self.action_space = gym.spaces.Tuple([gym.spaces.MultiDiscrete([self.num_node_types, self.max_location_size]) for _ in range(num_envs)])

        # Observation Space: String representing BT
        self.max_bt_length = 256  # Define a max length for encoding
        self.observation_space = gym.spaces.Tuple([gym.spaces.Text(max_length=self.max_bt_length) for _ in range(num_envs)])

        # Local Variables for Each Environment
        self.current_bt = ['' for _ in range(num_envs)]
        self.number_of_nodes_in_bt = [0 for _ in range(num_envs)]

        # Local Variables for Each Simulation
        self.sim_step = 0
        self.object_at_spawn = [0 for _ in range(num_envs)]
        self.object_found = [0 for _ in range(num_envs)]
        self.idle_step_count = [0 for _ in range(num_envs)]
        self.is_idled = [False for _ in range(num_envs)]
        self.is_task_success = [False for _ in range(num_envs)]
        self.elapsed_step = [0 for _ in range(num_envs)]

        # ROS2 Communication - Reward Related
        for i in range(num_envs):
            self.node.create_subscription(UInt8, f'/env_{i}/object_at_spawn', lambda msg, idx=i: self._set_attr(idx, 'object_at_spawn', msg.data), 10)
            self.node.create_subscription(UInt8, f'/env_{i}/object_found', lambda msg, idx=i: self._set_attr(idx, 'object_found', msg.data), 10)

    def _set_attr(self, idx, attr, value):
        """
        Update a specific environment's internal state synced with ROS2 topics.

        :param idx: Index of the environment to update.
        :param attr: Name of the attribute to set.
        :param value: New value received from ROS2.
        """
        setattr(self, attr, self.__getattribute__(attr)[:idx] + [value] + self.__getattribute__(attr)[idx+1:])

    def _create_sim(self):
        """
        Initialize IsaacSim and all scene managers required for the environment.

        This includes:
        - Simulation context and camera setup.
        - Scene and object placement.
        - ROS2-based battery manager.
        - Object group manager.
        - Robot driver manager.
        - BT status tracker.
        - Scene node for multi-agent control.
        """
        sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt = 1/30)
        self.sim = SimulationContext(sim_cfg)

        # Define simulation stepping
        self.sim_dt = self.sim.get_physics_dt()

        # Set main camera
        self.sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

        # Design scene
        scene_cfg = ManualBTSceneCfg(num_envs=args_cli.num_envs, env_spacing=args_cli.env_spacing)
        self.scene = InteractiveScene(scene_cfg)

        # Play the simulator
        self.sim.reset()
        print("[INFO]: Setup complete...")
        print(f"[INFO]: Currently running on {args_cli.device}")

        # Extract scene entities
        self.robot = self.scene["robot"]

        ### Initialize The Battery Manager ###
        battery_names = []
        for i in range(self.scene.num_envs):
            # Declare battery name for each robot
            battery_names.append(f'env_{i}')
            
        # Battery configuration
        discharge_rate = 0.05   # % per second
        charge_rate = 5.0       # % per second

        self.battery_manager = BatteryManager(battery_names, discharge_rate, charge_rate)

        ### Initialize The Object Group Manager ###
        object_group_names = []
        for env_id in range(self.scene.num_envs):
            # Declare object group name for each environment
            object_group_names.append(f'env_{env_id}')

        # Calculate opject position offset
        object_pos_offset = self.scene.env_origins.repeat([get_random_object_pos(self.scene.num_envs).shape[1],1,1])
        object_pos_offset = torch.transpose(object_pos_offset, 0, 1)

        self.object_pos_llist = get_random_object_pos(self.scene.num_envs, mode=self.object_pos_mode) + object_pos_offset
        self.object_group_manager = ObjectGroupManager(object_group_names, self.object_pos_llist, self.scene)

        ### Initialize The Robot Driver Manager ###
        self.driver_manager = RobotDriverManager(num_envs=self.scene.num_envs)
        self.joint_names = [
            "front_left_wheel_joint", "rear_left_wheel_joint",
            "front_right_wheel_joint", "rear_right_wheel_joint"
        ]

        ### Initialize BT Status Tracker ###
        self.bt_tracker = BTStatusTrackerNode(num_envs=self.scene.num_envs)
 
        ### Initialize Scene Node ###
        self.scene_node = SceneNode(self.scene.num_envs)

    def _reset_sim(self):
        """
        Reset the simulation state for all environments.

        This includes:
        - Resetting robot root position and orientation based on environment origins.
        - Resetting robot joint positions and velocities to default values.
        - Resetting battery levels using the battery manager.
        - Repositioning objects in the scene using the object group manager.
        """
        self.sim_step = 0
        self.object_at_spawn = [0 for _ in range(self.num_envs)]
        self.object_found = [0 for _ in range(self.num_envs)]
        self.idle_step_count = [0 for _ in range(self.num_envs)]
        self.is_idled = [False for _ in range(self.num_envs)]
        self.is_task_success = [False for _ in range(self.num_envs)]
        self.elapsed_step = [0 for _ in range(self.num_envs)]

        ### Reset Robot Root State ###
        robot_root_state = self.robot.data.default_root_state.clone()
        robot_root_state[:, :3] += self.scene.env_origins

        # initial position referenced by env
        robot_root_state[:, :3] += torch.tensor([0.0, 0.0, 0.7], device=args_cli.device)

        # initial orientation
        robot_root_state[:, 3:7] += torch.tensor(robot_rot, device=args_cli.device)

        self.robot.write_root_state_to_sim(robot_root_state)

        # Reset Robot Joint State
        joint_pos, joint_vel = (
            self.robot.data.default_joint_pos.clone(),
            self.robot.data.default_joint_vel.clone(),
        )
        self.robot.write_joint_state_to_sim(joint_pos, joint_vel)

        ### Reset Battery Level ###
        self.battery_manager.reset()

        ### Reset Object Position ###
        self.object_group_manager.repos(self.object_pos_llist)

    def _is_sim_terminated(self):
        """
        Determine whether the simulation should terminate.

        Termination conditions:
        - All environments idle for more than `idle_step_limit` steps.
        - Total simulation steps exceed `sim_step_limit`.

        :return: True if simulation should terminate, False otherwise.
        """
        output = False

        ### Condition 1: Idle ###
        # Update idle step count
        for env_id in range(self.num_envs):
            if self.bt_tracker.get_status(env_id) != 'RUNNING':
                self.idle_step_count[env_id] += 1
            else:
                # Reset if not idle consecutively
                self.idle_step_count[env_id] = 0
                self.is_idled[env_id] = False # This is rarely happened

            # A environment is considered terminated if idle consecutively for more than 200 simulation steps
            if self.idle_step_count[env_id] > self.idle_step_limit:
                self.is_idled[env_id] = True
                self.elapsed_step[env_id] = self.sim_step # Record elapse step of an environment
        
        # Terminate if all environment are idled
        if np.array(self.is_idled).all():
            output = True
        
        ### Condition 2: Timeout ###
        if self.sim_step > self.sim_step_limit:
            output = True

        return output
    
    def _BT_complexity(self, env_id):
        """
        Compute the complexity of the BT for a given environment.

        Complexity is based on:
        - Tree depth.
        - Number of nodes at each depth.
        - Logarithmic weighting for balance and branching factor.

        :param env_id: Index of the environment.
        :return: Floating point value representing the complexity score.
        """
        depth = 0
        complexity = 0
        depth_width = {}
        bt_string = self.current_bt[env_id]

        # First pass to compute depth_width
        for char in bt_string:
            if char == '(':
                depth += 1
            elif char == ')':
                depth -= 1
            elif char.isdigit():
                depth_width[depth - 1] = depth_width.get(depth - 1, 0) + 1
            else:
                depth_width[depth] = depth_width.get(depth, 0) + 1

        # Reset depth for second pass to compute complexity
        depth = 0
        for char in bt_string:
            if char == '(':
                depth += 1
            elif char == ')':
                depth -= 1
            elif char.isdigit():
                if depth > 0 and depth - 1 in depth_width and depth > 0:
                    complexity += depth * np.log(depth_width[depth - 1]) * np.log(depth)
            else:
                if depth in depth_width:
                    complexity += (depth + 1) * np.log(depth_width[depth]) * np.log(depth + 1)

        return complexity

    def _get_reward(self):
        """
        Compute the reward for each environment based on simulation outcomes.

        Reward is computed from:
        - Number of delivered and found objects.
        - Time bonus if the task is completed.
        - Penalty for battery death or complex BTs.

        :return: List of rewards for each environment.
        """
        rewards = []
        for env_id in range(self.num_envs):
            reward = 0

            ### Term 1: Number of delivered object ###
            reward += self.reward_weight[0] * self.object_at_spawn[env_id]

            ### Term 2: Number of found object ###
            reward += self.reward_weight[1] * self.object_found[env_id]

            ### Term 3: Time Bonus if task success ###
            if self.object_found[env_id] >= self.number_of_target_to_success:
                self.is_task_success[env_id] = True
            
            reward += self.reward_weight[2] * self.is_task_success[env_id] * self.sim_step_limit/self.elapsed_step[env_id]

            ### Term 4: Battery Dead Penalty ###
            reward += self.reward_weight[3] * (self.is_idled[env_id] and not(self.is_task_success[env_id]))

            ### Term 5: Tree Complexity Penalty ###
            reward += self.reward_weight[4] * self._BT_complexity(env_id)

            rewards.append(reward)

        return rewards

    def step(self, actions):
        """
        Execute one step for all environments with the given actions.

        :param actions: List of (node_type, node_location) for each environment.
        :return: Tuple of (observations, rewards, dones, infos) for all environments.
        """
        obs, rews, dones, infos = [], [], [], []

        self._reset_sim() # reset the simulation before running the BT

        # Modify each BT and run it
        bt_with_evaluation_node = []
        for env_id, action in enumerate(actions):
            node_type, node_location = action
            self.modify_bt(env_id, node_type, node_location)    # add node according to selected action
            bt_with_evaluation_node.append('(1H' + self.current_bt[env_id] + ')')   # add evaluation node

            done = False
            # Done if agent select to not expand the tree
            if node_type == 19:
                done = True
            elif 0 <= node_type <= 19:
                self.number_of_nodes_in_bt[env_id] += 1

            # Done if the number of nodes in BT exceed the limit
            if self.number_of_nodes_in_bt[env_id] > 50:
                done = True
            
            dones.append(done)

        obs = self.current_bt
        run_BTs(bt_with_evaluation_node) # run BTs

        # Run the simulation
        while simulation_app.is_running():
            # Publish Transformation Data
            pub_scene_data(self.scene.num_envs, self.base_node, self.scene)

            # Control motors via cmd_vel
            self.driver_manager.apply(self.robot, self.joint_names)
            self.robot.write_data_to_sim()

            # Spin BT Tracker Node
            rclpy.spin_once(self.bt_tracker, timeout_sec=0.0)

            # Perform step
            self.sim.step()
            
            # Update buffers
            self.scene.update(self.sim_dt)

            # Check if the simulation must be terminated
            if self._is_sim_terminated():
                break

        stop_BTs()  # stop all BT processes
        rews = self._get_reward()

        return obs, rews, dones, infos

    def reset(self):
        """
        Reset all environments to an empty BT state.

        :return: List of reset observations (initial BTs) for each environment.
        """
        self.current_bt = ['' for _ in range(self.num_envs)]
        self.number_of_nodes_in_bt = [0 for _ in range(self.num_envs)]
        return self.current_bt

    def modify_bt(self, env_id, node_type, node_location):
        """
        Modify the BT string of environment `env_id` by inserting a node at a valid location.

        :param env_id: Index of the environment.
        :param node_type: Type of node to insert (0–19).
        :param node_location: Index in the BT string to insert the node.
        """
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
        bt_string = self.current_bt[env_id][1:-1] if self.current_bt[env_id].startswith('(') else self.current_bt[env_id]

        if node is not None:
            # Generate list of valid insert positions
            valid_indices = [j for j in range(len(bt_string)+1) if j == len(bt_string) or not bt_string[j].isdigit()]
            if 0 <= node_location < len(valid_indices):
                insert_index = valid_indices[node_location]
                self.current_bt[env_id] = '(' + bt_string[:insert_index] + node + bt_string[insert_index:] + ')'
    
    def set_bt(self, env_id, bt_string):
        """
        Forcefully set the current BT string of an environment before modifying it.

        :param env_id: Index of the environment.
        :param bt_string: BT string to restore as state.
        """
        self.current_bt[env_id] = bt_string

###
#  Monte-Carlo Tree Search
###
import numpy as np
import random
import math

class MCTSNode:
    def __init__(self, state, env_id, parent=None, action=None):
        """
        Initialize an MCTSNode representing a Behavior Tree state.

        :param state: String representation of the current BT.
        :param env_id: ID of the environment the node is associated with.
        :param parent: Parent MCTSNode.
        :param action: Action taken from the parent to reach this node.
        """
        self.state = state                  # BT string for this node
        self.env_id = env_id                # Index of the environment
        self.parent = parent                # Parent node in the tree
        self.action = action                # Action used to reach this node
        self.children = []                  # List of children MCTSNodes
        self.visits = 0                     # Visit count for PUCT
        self.value = 0.0                    # Accumulated value from rollouts
        self.prior = 1.0                    # Prior probability from policy network

    def best_child(self, exploration_weight=1.0):
        """
        Return the best child based on PUCT formula.

        :param exploration_weight: PUCT exploration constant.
        :return: The selected best child node.
        """
        total_visits = sum(c.visits for c in self.children) + 1  # Total visit count of all children
        return max(
            self.children,
            key=lambda c: (c.value / (c.visits + 1e-6)) +
                          exploration_weight * c.prior * math.sqrt(total_visits) / (1 + c.visits)
        )

class ParallelMCTS:
    def __init__(self, env, policy_net, num_simulations=50, exploration_weight=1.0, device='cpu'):
        """
        Initialize the MCTS search structure.

        :param env: MultiBTEnv instance to simulate actions.
        :param policy_value_net: RvNN model used to guide search.
        :param num_simulations: Number of rollout iterations.
        :param exploration_weight: Controls exploration in PUCT.
        :param device: Device to run the model on ('cpu' or 'cuda').
        """
        self.env = env
        self.policy_net = policy_net.to(device)
        self.num_simulations = num_simulations
        self.exploration_weight = exploration_weight
        self.device = device
        self.root_visit_count = 0           # Track how many rollouts started from root

    def run_search(self, root_state, temperature=1.0):
        """
        Perform full MCTS search starting from a shared root state.

        :param root_state: Initial BT string for all environments.
        :param temperature: Temperature for softmax over visit counts.
        :return: Tuple:
                - best_actions: List of best (node_type, location) actions
                - action1_probs: List of numpy arrays [num_node_types]
                - action2_probs: List of numpy arrays [max_location_size]
        """
        roots = [MCTSNode(state=root_state, env_id=i) for i in range(self.env.num_envs)]
        max_locs = self.env.max_location_size
        num_types = self.env.num_node_types

        # Run Monte Carlo Tree Search
        for _ in range(self.num_simulations):
            leaves = [self.select(node) for node in roots]
            self.expand(leaves, add_dirichlet_noise=(self.root_visit_count == 0))
            rewards = self.simulate(leaves)
            for node, reward in zip(leaves, rewards):
                self.backpropagate(node, reward)
            self.root_visit_count += 1

        best_actions = []
        action1_probs_all = []
        action2_probs_all = []

        for root in roots:
            visit_matrix = np.zeros((num_types, max_locs), dtype=np.float32)
            for child in root.children:
                node_type, loc = child.action
                visit_matrix[node_type, loc] += child.visits

            # Extract best action
            best_action_idx = np.unravel_index(np.argmax(visit_matrix), visit_matrix.shape)
            best_actions.append(best_action_idx)

            if temperature == 0:
                # One-hot for training stability (greedy)
                action1_probs = np.zeros(num_types)
                action2_probs = np.zeros(max_locs)
                action1_probs[best_action_idx[0]] = 1.0
                action2_probs[best_action_idx[1]] = 1.0
            else:
                # Compute softmax separately
                visit_matrix_t = np.power(visit_matrix, 1.0 / temperature)

                action1_sums = np.sum(visit_matrix_t, axis=1)
                action2_sums = np.sum(visit_matrix_t, axis=0)

                action1_probs = action1_sums / (np.sum(action1_sums) + 1e-8)
                action2_probs = action2_sums / (np.sum(action2_sums) + 1e-8)

            action1_probs_all.append(action1_probs)
            action2_probs_all.append(action2_probs)

        return best_actions, action1_probs_all, action2_probs_all

    def select(self, node):
        """
        Traverse the tree using PUCT to select a leaf node for expansion.

        :param node: Root or intermediate node.
        :return: Selected leaf node.
        """
        while node.children:
            total_visits = sum(c.visits for c in node.children) + 1  # Avoid log(0)
            node = max(
                node.children,
                key=lambda c: (c.value / (c.visits + 1e-6)) +
                              self.exploration_weight * c.prior * math.sqrt(total_visits) / (1 + c.visits)
            )
        return node

    def expand(self, nodes, add_dirichlet_noise=False):
        """
        Expand each selected leaf node with a new child based on policy probabilities.

        :param nodes: List of leaf nodes to expand.
        :param add_dirichlet_noise: Whether to inject Dirichlet noise at root.
        """
        actions = []
        priors = []

        for node in nodes:
            # Get valid node locations from BT string
            bt_string = node.state[1:-1] if node.state.startswith('(') else node.state
            valid_locs = [j for j in range(len(bt_string)+1) if j == len(bt_string) or not bt_string[j].isdigit()]
            all_actions = [(nt, loc) for nt in range(self.env.num_node_types) for loc in range(len(valid_locs))]

            explored = {child.action for child in node.children}
            unexplored = [a for a in all_actions if a not in explored]

            if not unexplored:
                actions.append((19, 0))  # fallback stop
                priors.append(1.0)
            else:
                with torch.no_grad():
                    action1_probs, action2_probs = self.policy_net.predict_from_bt_string(node.state)
                    action1_probs = action1_probs.cpu().numpy()
                    action2_probs = action2_probs.cpu().numpy()

                if add_dirichlet_noise:
                    alpha = 0.3
                    epsilon = 0.25
                    dirichlet_noise1 = np.random.dirichlet([alpha] * len(action1_probs))
                    dirichlet_noise2 = np.random.dirichlet([alpha] * len(action2_probs))
                    action1_probs = (1 - epsilon) * action1_probs + epsilon * dirichlet_noise1
                    action2_probs = (1 - epsilon) * action2_probs + epsilon * dirichlet_noise2

                combined_probs = np.array([
                    action1_probs[nt] * action2_probs[loc] for nt, loc in unexplored
                ])
                combined_probs /= (np.sum(combined_probs) + 1e-8)

                selected_idx = np.random.choice(len(unexplored), p=combined_probs)
                selected_action = unexplored[selected_idx]
                actions.append(selected_action)
                priors.append(combined_probs[selected_idx])

                # Restore correct state before modifying BT
                self.env.set_bt(env_id=node.env_id, bt_string=node.state)
                self.env.modify_bt(node.env_id, selected_action[0], selected_action[1])

        # Now that env.current_bt reflects correct BTs, run them
        obs, rews, _, _ = self.env.step(actions)

        for node, action, new_state, reward, prior in zip(nodes, actions, obs, rews, priors):
            child = MCTSNode(state=new_state, env_id=node.env_id, parent=node, action=action)
            child.value = reward
            child.prior = prior
            node.children.append(child)


    def simulate(self, nodes):
        """
        Get reward directly from environment rather than estimating with value head.

        :param nodes: List of nodes after expansion.
        :return: List of scalar rewards.
        """
        return [node.children[-1].value for node in nodes]  # Use most recent child value

    def backpropagate(self, node, reward):
        """
        Propagate the reward up from the leaf node to the root.

        :param node: Final node after simulation.
        :param reward: Reward to distribute up the path.
        """
        while node:
            node.visits += 1
            node.value += reward  # Add reward to value
            node = node.parent

###
#  Neural Network for Policy
###
import torch
import torch.nn as nn
import torch.optim as optim
from torch.nn.utils.rnn import pad_sequence

# ----------- Behavior Dictionary (char -> node type index) ----------- #
            # Flow Control
node_dict = { 0 : '0', # sequence_node
              1 : '1', # selector_node
              2 : '2', # parallel_node
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

# === Inverted character -> type index ===
char_to_node_type = {v: k for k, v in node_dict.items() if v is not None}

# === Composite node types ===
composite_chars = {'0', '1', '2'}

class TreeNode:
    """
    Simple recursive tree structure to represent a Behavior Tree (BT) node.
    
    Args:
        name (str): Node name for readability.
        node_type (int): Index representing the type of node (from node_dict).
        children (list): List of child TreeNode objects.
    """
    def __init__(self, name, node_type, children=None):
        self.name = name
        self.node_type = node_type
        self.children = children if children else []

def string_to_tree_node(tree_string, node_id=0):
    """
    Parse a BT string representation directly into a TreeNode-based tree.
    
    Args:
        tree_string (str): A BT string like "(0a(1bC))".
        node_id (int): Used for naming composite nodes uniquely (optional).
    
    Returns:
        TreeNode: Root of the parsed tree.
    """
    assert tree_string[0] == '(', "[Error] Tree must start with '('"
    assert tree_string[-1] == ')', "[Error] Tree must end with ')'"

    # First character after '(' is the root node type
    node_type_char = tree_string[1]
    
    if node_type_char in composite_chars:
        # It's a flow control (composite) node
        node_type = char_to_node_type[node_type_char]
        name = f"composite_{node_id}"
        node_id += 1
    elif node_type_char in char_to_node_type:
        # It's a single behavior/condition node
        node_type = char_to_node_type[node_type_char]
        name = f"leaf_{node_type_char}"
        return TreeNode(name, node_type)
    else:
        raise ValueError(f"[Error] Unknown node type char: {node_type_char}")

    # Recursively parse children
    children = []
    i = 2
    while i < len(tree_string) - 1:
        c = tree_string[i]
        if c == '(':
            # Start of a subtree
            depth = 1
            start = i
            while depth != 0:
                i += 1
                if tree_string[i] == '(':
                    depth += 1
                elif tree_string[i] == ')':
                    depth -= 1
            subtree = tree_string[start:i+1]
            child = string_to_tree_node(subtree, node_id)
            children.append(child)
            i += 1
        elif c in char_to_node_type:
            # Leaf node (single character)
            child = TreeNode(f"leaf_{c}", char_to_node_type[c])
            children.append(child)
            i += 1
        else:
            print("[Error] Skipping unknown char:", c)
            i += 1

    return TreeNode(name, node_type, children)

class RvNN(nn.Module):
    """
    Recursive Neural Network for encoding Behavior Trees and predicting
    two discrete action probability distributions:
        1. Node type to expand
        2. Node location to apply expansion

    Args:
        node_type_vocab_size (int): Total number of node types (typically 20).
        embed_size (int): Size of the embedding vector for each node.
        hidden_size (int): Size of the hidden vector in recursive processing.
        action1_size (int): Output size for action 1 (node type prediction).
        action2_size (int): Output size for action 2 (location prediction).
    """
    def __init__(self, node_type_vocab_size, embed_size, hidden_size, action1_size, action2_size):
        super(RvNN, self).__init__()
        self.node_embedding = nn.Embedding(node_type_vocab_size, embed_size)
        self.W_c = nn.Linear(embed_size + hidden_size, hidden_size)
        self.activation = nn.Tanh()

        self.output_action1 = nn.Linear(hidden_size, action1_size)
        self.output_action2 = nn.Linear(hidden_size, action2_size)

        self.child_gru = nn.GRU(hidden_size, hidden_size, batch_first=True)

    def forward(self, node):
        """
        Forward pass: encode a TreeNode structure recursively.

        Args:
            node (TreeNode): Root node of the tree.

        Returns:
            torch.Tensor: Hidden vector encoding of the tree.
        """
        return self._encode_node(node)

    def _encode_node(self, node):
        """
        Internal method to recursively encode a TreeNode using a child-GRU.

        Args:
            node (TreeNode): A node to process.

        Returns:
            torch.Tensor: Encoded hidden state for this subtree.
        """
        if not node.children:
            embed = self.node_embedding(torch.tensor([node.node_type]))
            zero_pad = torch.zeros(self.W_c.in_features - embed.size(1), device=embed.device)
            h = self.activation(self.W_c(torch.cat([embed.squeeze(0), zero_pad])))
            return h

        # Encode children recursively
        child_states = [self._encode_node(child).unsqueeze(0) for child in node.children]
        child_seq = torch.cat(child_states, dim=0).unsqueeze(0)  # shape: [1, num_children, hidden_size]

        _, last_hidden = self.child_gru(child_seq)  # shape: [1, 1, hidden_size]
        child_encoding = last_hidden.squeeze(0).squeeze(0)  # shape: [hidden_size]

        embed = self.node_embedding(torch.tensor([node.node_type]))
        concat = torch.cat([embed.squeeze(0), child_encoding])  # [embed + child_hidden]
        h = self.activation(self.W_c(concat))
        return h

    def predict_from_bt_string(self, bt_string):
        """
        Predict probability distributions for two actions from a BT string.

        Args:
            bt_string (str): A string representing a Behavior Tree (e.g., "(0a(1bC)").

        Returns:
            Tuple[Tensor, Tensor]: Probabilities for action1 (node type), action2 (location).
        """
        root = string_to_tree_node(bt_string)
        h = self.forward(root)

        action1_logits = self.output_action1(h)
        action2_logits = self.output_action2(h)

        action1_probs = torch.softmax(action1_logits, dim=0)
        action2_probs = torch.softmax(action2_logits, dim=0)

        return action1_probs, action2_probs

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
    bt_string = current_bt[1:-1]  # Remove the outer parentheses

    if node != None:
        valid_indices = []

        # Iterate over all potential insertion positions (0 to len(s))
        valid_indices = [j for j in range(len(bt_string)+1) if j == len(bt_string) or not bt_string[j].isdigit()]

        if 0 <=  node_location < len(valid_indices):
            # Get the actual index in the string where we want to insert the char.
            insert_index = valid_indices[node_location]

            # Return the new string with the character inserted.
            return '(' + bt_string[:insert_index] + node + bt_string[insert_index:] + ')'
        
    return current_bt

def dataset_generation(num_envs, policy_net, device='cuda:0'):
    policy_net = policy_net.to(device)

    env = MultiBTEnv(num_envs=num_envs)
    mcts = ParallelMCTS(env, policy_net, num_simulations=50, exploration_weight=1.0, device=device)

    initial_tree = ''
    
import torch
from torch.utils.data import Dataset

class BTDataset(Dataset):
    def __init__(self, bt_strings, action1_probs, action2_probs):
        """
        :param bt_strings: List of behavior tree strings
        :param action1_probs: List of node type distributions (numpy or list)
        :param action2_probs: List of location distributions (numpy or list)
        """
        self.bt_strings = bt_strings
        self.action1_probs = action1_probs
        self.action2_probs = action2_probs

    def __len__(self):
        return len(self.bt_strings)

    def __getitem__(self, idx):
        # You can convert numpy arrays to torch.Tensor here if needed
        return (
            self.bt_strings[idx],
            torch.tensor(self.action1_probs[idx], dtype=torch.float32),
            torch.tensor(self.action2_probs[idx], dtype=torch.float32)
        )

if __name__ == "__main__":
    # Example Behavior Tree string
    bt_str = "(0a(1bC))"  # Sequence node → a + (Selector → b, C)

    # Instantiate the model
    model = RvNN(
        node_type_vocab_size=20,  # 0 to 19
        embed_size=64,
        hidden_size=128,
        action1_size=20,          # Number of node types to choose from
        action2_size=50           # Max insertion locations
    )

    # Make prediction from BT string
    probs1, probs2 = model.predict_from_bt_string(bt_str)

    # Print outputs
    print("Action1 probs (Node Type):", probs1)
    print("Action2 probs (Location):", probs2)


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
