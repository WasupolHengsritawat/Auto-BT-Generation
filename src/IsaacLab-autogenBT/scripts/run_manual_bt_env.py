# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""This script is used to simulated and visualize the robot behavior given the manually created BT."""

"""Launch Isaac Sim Simulator first."""

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

cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT')

##
# Pre-defined configs
##
from omni.isaac.core.utils.extensions import enable_extension

from assets.jackal_ur5 import JACKAL_UR5_CFG  # isort:skip
from omnigraph import create_controller_omnigraph
from ros2_publisher import pub_scene_data, SceneNode
from battery import BatteryManager

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")
enable_extension("omni.isaac.wheeled_robots")

# Calculate initial rotation
map_rot = Rotation.from_euler('xyz', [90, 0, 0], degrees=True).as_quat()
robot_rot = Rotation.from_euler('xyz', [-630, -400, 60], degrees=True).as_quat()

# Possible target spawn point
target_spawn_pool = [[ 14.0, -4.0,  0.1],
                     [ 22.0, -1.0,  0.1],
                     [ 19.5,  7.7,  0.1],
                     [ 20.5, 13.0,  0.1],
                     [ 23.0, 21.0,  0.1],
                     [  9.0, 24.5,  0.1],
                     [ -2.0, 23.2,  0.1],
                     [ -1.5, 13.3,  0.1],
                     [ -2.6,  7.7,  0.1],
                     [  8.0,  6.7,  0.1]]

# target_spawn_pool = [[ 14.0, -4.0,  0.1],
#                      [ 22.0, -1.0,  0.1],
#                      [ 19.5,  7.7,  0.1],
#                      [ -2.6,  7.7,  0.1],
#                      [  8.0,  6.7,  0.1]]

target_spawn_ind = np.random.default_rng().permutation(len(target_spawn_pool))


# target_spawn_pool[target_spawn_ind[i]] # increment in i

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

    # target object
    # With dynamics 
    target_1: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Target_1",
        spawn=sim_utils.CuboidCfg(
            size=[0.06] * 3,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
        init_state=RigidObjectCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[0]]),
    )

    target_2: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Target_2",
        spawn=sim_utils.CuboidCfg(
            size=[0.06] * 3,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
        init_state=RigidObjectCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[1]]),
    )

    target_3: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Target_3",
        spawn=sim_utils.CuboidCfg(
            size=[0.06] * 3,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
        init_state=RigidObjectCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[2]]),
    )

    target_4: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Target_4",
        spawn=sim_utils.CuboidCfg(
            size=[0.06] * 3,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
        init_state=RigidObjectCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[3]]),
    )

    target_5: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Target_5",
        spawn=sim_utils.CuboidCfg(
            size=[0.06] * 3,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
        init_state=RigidObjectCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[4]]),
    )

    # Only Kinematics

    # target_1: AssetBaseCfg = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Target_1", 
    #     spawn=sim_utils.CuboidCfg(size=[0.06] * 3,
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
    #     init_state = AssetBaseCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[0]])
    # )

    # target_2: AssetBaseCfg = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Target_2", 
    #     spawn=sim_utils.CuboidCfg(size=[0.06] * 3,
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
    #     init_state = AssetBaseCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[1]])
    # )

    # target_3: AssetBaseCfg = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Target_3", 
    #     spawn=sim_utils.CuboidCfg(size=[0.06] * 3,
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
    #     init_state = AssetBaseCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[2]])
    # )

    # target_4: AssetBaseCfg = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Target_4", 
    #     spawn=sim_utils.CuboidCfg(size=[0.06] * 3,
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
    #     init_state = AssetBaseCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[3]])
    # )

    # target_5: AssetBaseCfg = AssetBaseCfg(
    #     prim_path="{ENV_REGEX_NS}/Target_5", 
    #     spawn=sim_utils.CuboidCfg(size=[0.06] * 3,
    #     visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
    #     init_state = AssetBaseCfg.InitialStateCfg(pos = target_spawn_pool[target_spawn_ind[4]])
    # )


    # articulation
    robot: ArticulationCfg = JACKAL_UR5_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # # gripper contact sensor
    # contact_forces = ContactSensorCfg(
    #     prim_path="{ENV_REGEX_NS}/Robot/UR5/Robotiq_Hand_E/right_gripper", update_period=0.0, history_length=6, debug_vis=True
    # )


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]
    target_1 = scene["target_1"]
    target_2 = scene["target_2"]
    target_3 = scene["target_3"]
    target_4 = scene["target_4"]
    target_5 = scene["target_5"]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0

    # Initialize ROS2 node
    base_node = SceneNode(args_cli.num_envs)

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
            target_1_root_state = target_1.data.default_root_state.clone() - target_1.data.default_root_state.clone()
            target_2_root_state = target_2.data.default_root_state.clone() - target_2.data.default_root_state.clone()
            target_3_root_state = target_3.data.default_root_state.clone() - target_3.data.default_root_state.clone()
            target_4_root_state = target_4.data.default_root_state.clone() - target_4.data.default_root_state.clone()
            target_5_root_state = target_5.data.default_root_state.clone() - target_5.data.default_root_state.clone()

            target_1_root_state[:, :3] += scene.env_origins
            target_2_root_state[:, :3] += scene.env_origins
            target_3_root_state[:, :3] += scene.env_origins
            target_4_root_state[:, :3] += scene.env_origins
            target_5_root_state[:, :3] += scene.env_origins

            target_1_root_state[:, 3:7] += torch.tensor([1.0, 0.0, 0.0, 0.0],device=args_cli.device)
            target_2_root_state[:, 3:7] += torch.tensor([1.0, 0.0, 0.0, 0.0],device=args_cli.device)
            target_3_root_state[:, 3:7] += torch.tensor([1.0, 0.0, 0.0, 0.0],device=args_cli.device)
            target_4_root_state[:, 3:7] += torch.tensor([1.0, 0.0, 0.0, 0.0],device=args_cli.device)
            target_5_root_state[:, 3:7] += torch.tensor([1.0, 0.0, 0.0, 0.0],device=args_cli.device)

            target_spawn_ind = np.random.default_rng().permutation(len(target_spawn_pool))

            target_1_root_state[:, :3] += torch.tensor(target_spawn_pool[target_spawn_ind[0]],device=args_cli.device)
            target_2_root_state[:, :3] += torch.tensor(target_spawn_pool[target_spawn_ind[1]],device=args_cli.device)
            target_3_root_state[:, :3] += torch.tensor(target_spawn_pool[target_spawn_ind[2]],device=args_cli.device)
            target_4_root_state[:, :3] += torch.tensor(target_spawn_pool[target_spawn_ind[3]],device=args_cli.device)
            target_5_root_state[:, :3] += torch.tensor(target_spawn_pool[target_spawn_ind[4]],device=args_cli.device)

            target_1.write_root_state_to_sim(target_1_root_state)
            target_2.write_root_state_to_sim(target_2_root_state)
            target_3.write_root_state_to_sim(target_3_root_state)
            target_4.write_root_state_to_sim(target_4_root_state)
            target_5.write_root_state_to_sim(target_5_root_state)

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

    battery_names = []
    for i in range(args_cli.num_envs):

        # Create ROS2 omnigraph
        create_controller_omnigraph(i)

        # Declare battery name for each robot
        battery_names.append(f'robot_{i}')

    # Battery configuration
    discharge_rate = 0.05   # % per second
    charge_rate = 5.0       # % per second

    # Initialize ROS2 node
    rclpy.init()

    # Initialize the battery manager
    manager = BatteryManager(battery_names, discharge_rate, charge_rate)
    
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    try:
        # Run the simulator
        run_simulator(sim, scene)

    finally:
        # Gracefully stop all battery nodes
        manager.stop()
        print("Battery nodes stopped.")

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()