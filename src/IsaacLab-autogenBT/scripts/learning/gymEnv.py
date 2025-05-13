###
#  Simulation Configuration
###
import torch
import numpy as np
import sys
import os
import rclpy
from scipy.spatial.transform import Rotation

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass

# Import local files
# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
simulation_dir = os.path.abspath(os.path.join(project_root, "scripts", "simulation"))
bt_dir = os.path.abspath(os.path.join(project_root, "scripts", "bt"))

# Add it to sys.path 
sys.path.insert(0, project_root)
sys.path.insert(0, script_dir)
sys.path.insert(0, simulation_dir)
sys.path.insert(0, bt_dir)

from assets.jackal_ur5 import JACKAL_UR5_CFG
from ros2_nodes.ros2_scene_publisher import pub_scene_data, SceneNode
from ros2_nodes.ros2_battery import BatteryManager
from ros2_nodes.ros2_object import ObjectGroupManager
from ros2_nodes.ros2_drive import RobotDriverManager
from ros2_nodes.ros2_bt_tracker import BTStatusTrackerNode
from bt_manager import run_BTs, stop_BTs

def get_random_object_pos(num_env, device, number_of_objects = 5,mode = 'different'):
    """
    Generate random object target positions from a predefined spawn pool.

    :param num_env: Number of parallel environments.
    :param mode: 'different' assigns a unique set of positions to each environment,
                 'same' assigns the same set to all.
    :return: A tensor of shape (num_env, 5, 3) containing randomly selected target positions.
    """
    number_of_objects = 8

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
                     [  8.0,  6.7,  0.1]],device=device)
    
    if mode == 'different': target_pos = torch.cat([target_spawn_pool[torch.randperm(len(target_spawn_pool))[:number_of_objects]].unsqueeze(0) for _ in range(num_env)], dim=0)
    elif mode == 'same'   : target_pos = torch.cat([target_spawn_pool[torch.randperm(len(target_spawn_pool))[:number_of_objects]].unsqueeze(0)] * num_env, dim=0)
    
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
#  Gym Environment
###
import torch
import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from tqdm import tqdm

class MultiBTEnv(gym.Env):
    def __init__(self, node_dict, 
                 nodes_limit, num_envs, 
                 simulation_app = None, 
                 number_of_target_to_success = 5, 
                 number_of_spawned_objects = 5, 
                 sim_step_limit = 200000, 
                 object_pos_mode = 'different', 
                 env_spacing = None, 
                 device = None, 
                 verbose = False):
        """
        Initialize multiple BT environments and connect to ROS2 simulation.

        :param num_envs: Number of parallel environments (agents).
        """
        super().__init__()
        self.node_dict = node_dict
        self.num_envs = num_envs
        self.env_spacing = env_spacing
        self.device = device
        self.simulation_app = simulation_app
        self.verbose = verbose

        self.object_pos_mode = object_pos_mode  # 'same' if want all environment object positions to be the same
                                                # 'different' for otherwise
        self.idle_step_limit = 200
        self.sim_step_limit = sim_step_limit
        self.nodes_limit = nodes_limit
        self.number_of_target_to_success = number_of_target_to_success
        self.number_of_spawned_objects = number_of_spawned_objects

        self.reward_weight = [ 100,     # Number of delivered objects term
                                70,     # Number of founded objects term
                               100,     # Bonus time reward if success term
                              -250,     # Battery dead penalty term 
                                -0.25]  # Tree complexity penalty term

        if self.simulation_app is not None:
            self._create_sim()  # Create and initialize the simulation

            # ROS2 Communication - Reward Related
            for i in range(num_envs):
                self.node.create_subscription(UInt8, f'/env_{i}/object_at_spawn', lambda msg, idx=i: self._set_attr(idx, 'object_at_spawn', msg.data), 10)
                self.node.create_subscription(UInt8, f'/env_{i}/object_found', lambda msg, idx=i: self._set_attr(idx, 'object_found', msg.data), 10)

            # Local Variables for Each Simulation
            self.sim_step = 0
            self.object_at_spawn = [0 for _ in range(num_envs)]
            self.object_found = [0 for _ in range(num_envs)]
            self.idle_step_count = [0 for _ in range(num_envs)]
            self.is_idled = [False for _ in range(num_envs)]
            self.is_task_success = [False for _ in range(num_envs)]
            self.elapsed_step = [0 for _ in range(num_envs)]

        # Action Space: (Node Type, Node Location)
        self.num_node_types = len(self.node_dict.items())  # 20 possible node types
        self.max_location_size = (2 * self.nodes_limit - 1) + 1 # Maximum possible locations for child nodes + parent node
        self.action_space = gym.spaces.Tuple([gym.spaces.MultiDiscrete([self.num_node_types, self.max_location_size]) for _ in range(num_envs)])

        # Observation Space: String representing BT
        self.max_bt_length = 256  # Define a max length for encoding
        self.observation_space = gym.spaces.Tuple([gym.spaces.Text(max_length=self.max_bt_length) for _ in range(num_envs)])

        # Local Variables for Each Environment
        self.current_bt = ['' for _ in range(num_envs)]
        self.number_of_nodes_in_bt = [0 for _ in range(num_envs)]

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
        rclpy.init()
        self.node = Node('multi_bt_env')

        sim_cfg = sim_utils.SimulationCfg(device=self.device, dt = 1/30)
        self.sim = SimulationContext(sim_cfg)

        # Define simulation stepping
        self.sim_dt = self.sim.get_physics_dt()

        # Set main camera
        self.sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

        # Design scene
        scene_cfg = ManualBTSceneCfg(num_envs=self.num_envs, env_spacing=self.env_spacing)
        self.scene = InteractiveScene(scene_cfg)

        # Play the simulator
        self.sim.reset()
        print("[INFO]: Setup complete...")
        print(f"[INFO]: Currently running on {self.device}")

        # Extract scene entities
        self.robot = self.scene["robot"]

        ### Initialize The Battery Manager ###
        battery_names = []
        for i in range(self.scene.num_envs):
            # Declare battery name for each robot
            battery_names.append(f'env_{i}')
            
        # Battery configuration
        discharge_rate = 0.15    # % per second
        charge_rate = 5.0        # % per second

        self.battery_manager = BatteryManager(battery_names, discharge_rate, charge_rate)

        ### Initialize The Object Group Manager ###
        object_group_names = []
        for env_id in range(self.scene.num_envs):
            # Declare object group name for each environment
            object_group_names.append(f'env_{env_id}')

        # Calculate opject position offset
        object_pos_offset = self.scene.env_origins.repeat([get_random_object_pos(self.scene.num_envs, number_of_objects=self.number_of_spawned_objects, device=self.device).shape[1],1,1])
        object_pos_offset = torch.transpose(object_pos_offset, 0, 1)

        self.object_pos_llist = get_random_object_pos(self.scene.num_envs, number_of_objects=self.number_of_spawned_objects, mode=self.object_pos_mode, device=self.device) + object_pos_offset
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
        robot_root_state[:, :3] += torch.tensor([0.0, 0.0, 0.7], device=self.device)

        # initial orientation
        robot_root_state[:, 3:7] += torch.tensor(robot_rot, device=self.device)

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
            if self.idle_step_count[env_id] > self.idle_step_limit and not self.is_idled[env_id]:
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
            ### Check if the task is success ###
            if self.object_found[env_id] >= self.number_of_target_to_success:
                self.is_task_success[env_id] = True

            ### Term 1: Number of delivered object ###
            reward += self.reward_weight[0] * self.object_at_spawn[env_id]

            ### Term 2: Number of found object ###
            reward += self.reward_weight[1] * self.object_found[env_id]

            ### Term 3: Time Bonus if task success ###
            reward += self.reward_weight[2] * self.is_task_success[env_id] * self.sim_step_limit/(self.elapsed_step[env_id] + 1e-15)

            ### Term 4: Battery Dead Penalty ###
            reward += self.reward_weight[3] * (self.is_idled[env_id] and not(self.is_task_success[env_id]))

            ### Term 5: Tree Complexity Penalty ###
            reward += self.reward_weight[4] * self.is_task_success[env_id] * self._BT_complexity(env_id)

            rewards.append(reward)

        if self.verbose:
            print(f"[INFO] Rewards: {rewards}")
            print(f"[INFO] Object at spawn: {self.object_at_spawn}")
            print(f"[INFO] Object found: {self.object_found}")
            print(f"[INFO] Is task success: {self.is_task_success}")
            print(f"[INFO] Is idled: {self.is_idled}")
            print(f"[INFO] Elapsed step: {self.elapsed_step}")
        
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
            if node_type == self.num_node_types - 1:
                done = True
            elif 0 <= node_type <= self.num_node_types - 1:
                self.number_of_nodes_in_bt[env_id] += 1

            # Done if the number of nodes in BT exceed the limit
            if self.number_of_nodes_in_bt[env_id] > self.nodes_limit:
                done = True
            
            dones.append(done)

        obs = self.current_bt
        run_BTs(bt_with_evaluation_node, number_of_target_to_success = self.number_of_target_to_success) # run BTs

        pbar = tqdm(total=self.sim_step_limit)

        # Run the simulation
        while self.simulation_app.is_running():
            # Publish Transformation Data
            pub_scene_data(self.scene.num_envs, self.base_node, self.scene)

            # Control motors via cmd_vel
            self.driver_manager.apply(self.robot, self.joint_names)
            self.robot.write_data_to_sim()

            # Spin BT Tracker Node
            try:
                rclpy.spin_once(self.bt_tracker, timeout_sec=0.0)
            except rclpy.executors.ExternalShutdownException:
                print("[INFO] Spin exited because ROS2 shutdown detected.")

            # Perform step
            self.sim.step()

            # Increment counter
            self.sim_step += 1

            # Update the progress bar
            pbar.update(1)
            
            # Update buffers
            self.scene.update(self.sim_dt)

            # Check if the simulation must be terminated
            if self._is_sim_terminated():
                break

        pbar.close()

        stop_BTs()  # stop all BT processes
        rews = self._get_reward()

        return obs, rews, dones, infos
    
    def step_without_sim(self, actions):
        obs, rews, dones, infos = [], [], [], []
        
        for env_id, action in enumerate(actions):
            # Modify each BT
            node_type, node_location = action
            self.modify_bt(env_id, node_type, node_location)    # add node according to selected action

            done = False
            # Done if agent select to not expand the tree
            if node_type == self.num_node_types - 1:
                done = True
            elif 0 <= node_type <= self.num_node_types - 1:
                self.number_of_nodes_in_bt[env_id] += 1

            # Done if the number of nodes in BT exceed the limit
            if self.number_of_nodes_in_bt[env_id] > self.nodes_limit:
                done = True
            
            dones.append(done)

        obs = self.current_bt

        return obs, rews, dones, infos
    
    def evaluate_bt_in_sim(self):
        obs, rews, dones, infos = [], [], [], []

        self._reset_sim() # reset the simulation before running the BT

        # Modify each BT and run it
        bt_with_evaluation_node = []
        for env_id in range(self.num_envs):
            bt_with_evaluation_node.append('(1H' + self.current_bt[env_id] + ')')   # add evaluation node

        run_BTs(bt_with_evaluation_node, number_of_target_to_success = self.number_of_target_to_success) # run BTs

        pbar = tqdm(total=self.sim_step_limit)

        # Run the simulation
        while self.simulation_app.is_running():
            # Publish Transformation Data
            pub_scene_data(self.scene.num_envs, self.scene_node, self.scene)

            # Control motors via cmd_vel
            self.driver_manager.apply(self.robot, self.joint_names)
            self.robot.write_data_to_sim()

            # Spin BT Tracker Node
            try:
                rclpy.spin_once(self.bt_tracker, timeout_sec=0.0)
            except rclpy.executors.ExternalShutdownException:
                print("[INFO] Spin exited because ROS2 shutdown detected.")

            # Perform step
            self.sim.step()

            # Increment counter
            self.sim_step += 1

            # Update the progress bar
            pbar.update(1)
            
            # Update buffers
            self.scene.update(self.sim_dt)

            # Check if the simulation must be terminated
            if self._is_sim_terminated():
                self.driver_manager.reset()
                self.driver_manager.apply(self.robot, self.joint_names)
                self.robot.write_data_to_sim()
                self.sim.step()
                self.scene.update(self.sim_dt)
                break

        pbar.close()

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
        :param node_type: Type of node to insert (0-19).
        :param node_location: Index in the BT string to insert the node.
        """
        node = self.node_dict[node_type]
        bt_string = self.current_bt[env_id]

        if self.current_bt[env_id] == '':
            # If BT is empty, insert the node at the beginning
            self.current_bt[env_id] = node
            if node is None:
                self.current_bt[env_id] = ''

        if node is not None:
            # If node location is 0 and node type is a flow control node, we add it as a parent node
            if node_location == 0 and node_type in [0, 1, 2]:
                self.current_bt[env_id] = f'({node_type}' + bt_string + ')'
            else:   
                # Generate list of valid insert positions
                valid_indices = [j for j in range(1,len(bt_string)) if j == len(bt_string) or not bt_string[j].isdigit()]

                if 1 <= node_location < len(valid_indices) + 1:
                    insert_index = valid_indices[node_location - 1]
                    self.current_bt[env_id] = bt_string[:insert_index] + node + bt_string[insert_index:]
    
    def set_bt(self, env_id, bt_string):
        """
        Forcefully set the current BT string of an environment before modifying it.

        :param env_id: Index of the environment.
        :param bt_string: BT string to restore as state.
        """
        self.current_bt[env_id] = bt_string
