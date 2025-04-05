# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script is used to simulated and visualize the robot behavior given the manually created BT."""

"""Launch Isaac Sim Simulator first."""

# Launch App ==========================================================================================
import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Run simple simulation to demonstrate robot behavior given the manually created BT.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--env_spacing", type=int, default=40, help="Space between environments.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

# Simulation Config ===================================================================================
import torch
import numpy as np
import sys
import os
import rclpy
import code 
from scipy.spatial.transform import Rotation

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR
from omni.isaac.lab.sensors import ContactSensorCfg, FrameTransformerCfg
from omni.isaac.core.utils.extensions import enable_extension

# Import local files
cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT')
sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT/scripts')

from assets.jackal_ur5 import JACKAL_UR5_CFG
from utils.omnigraph import create_controller_omnigraph
from ros2_nodes.ros2_scene_publisher import pub_scene_data, SceneNode
from ros2_nodes.ros2_battery import BatteryManager
from ros2_nodes.ros2_object import ObjectGroupManager

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.wheeled_robots")

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
        spawn=sim_utils.UsdFileCfg(usd_path=f"{cwd}/src/IsaacLab-autogenBT/assets/cave_map.usd", scale=[0.0004, 0.0005, 0.0004]),
        init_state=AssetBaseCfg.InitialStateCfg(pos=[-7, 27, 0], rot=[map_rot[3], map_rot[0], map_rot[1], map_rot[2]])
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # articulation
    robot: ArticulationCfg = JACKAL_UR5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

# Run Simulation ======================================================================================
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]

    # battery init
    battery_names = []
    for i in range(scene.num_envs):
        # Create ROS2 omnigraph
        create_controller_omnigraph(i)

        # Declare battery name for each robot
        battery_names.append(f'env_{i}')

    # Battery configuration
    discharge_rate = 0.05   # % per second
    charge_rate = 5.0       # % per second

    # Initialize the battery manager
    battery_manager = BatteryManager(battery_names, discharge_rate, charge_rate)
    
    # target init
    # Initialize the object group manager
    object_group_names = []
    for env_id in range(scene.num_envs):
        # Declare object group name for each environment
        object_group_names.append(f'env_{env_id}')

    # Calculate opject position offset
    object_pos_offset = scene.env_origins.repeat([get_random_object_pos(scene.num_envs).shape[1],1,1])
    object_pos_offset = torch.transpose(object_pos_offset, 0, 1)

    object_group_manager = ObjectGroupManager(object_group_names, get_random_object_pos(scene.num_envs) + object_pos_offset,scene)

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0

    # Initialize ROS2 node
    base_node = SceneNode(scene.num_envs)

    # Simulation loop
    while simulation_app.is_running():
        # Publish Transformation Data
        pub_scene_data(scene.num_envs, base_node, scene)

        # Reset
        if count % 400000 == 0:
            # Reset Counter
            count = 0

            # Reset Robot Root state ====================================================
            robot_root_state = robot.data.default_root_state.clone()
            robot_root_state[:, :3] += scene.env_origins

            # initial position referenced by env
            robot_root_state[:, :3] += torch.tensor([0.0, 0.0, 0.7],device=args_cli.device)

            # initial orientation
            robot_root_state[:, 3:7] += torch.tensor(robot_rot,device=args_cli.device)

            robot.write_root_state_to_sim(robot_root_state)

            # Reset Robot Joint State
            joint_pos, joint_vel = (
                robot.data.default_joint_pos.clone(),
                robot.data.default_joint_vel.clone(),
            )
            robot.write_joint_state_to_sim(joint_pos,joint_vel)

            # Random New Target Spawn Location =========================================
            object_group_manager.repos(get_random_object_pos(scene.num_envs) + object_pos_offset)

            # Reset Battery Level ======================================================
            battery_manager.reset()

            # Reset Scene ==============================================================
            scene.reset()
            print("[INFO]: Resetting robot state...")

        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)

def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device, dt = 0.01)
    sim = SimulationContext(sim_cfg)

    # Set main camera
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    # Design scene
    scene_cfg = ManualBTSceneCfg(num_envs=args_cli.num_envs, env_spacing=args_cli.env_spacing)
    scene = InteractiveScene(scene_cfg)

    # Initialize ROS2 node
    rclpy.init()

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    try:
        # Run the simulator
        run_simulator(sim, scene)
    finally:
        # Gracefully stop all battery nodes
        rclpy.shutdown()

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()