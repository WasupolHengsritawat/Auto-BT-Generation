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
parser.add_argument("--env_spacing", type=int, default=70, help="Space between environments.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg, RigidObject
from omni.isaac.lab.scene import InteractiveScene, InteractiveSceneCfg
from omni.isaac.lab.sim import SimulationContext
from omni.isaac.lab.utils import configclass
from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR
from omni.isaac.lab.sensors import ContactSensorCfg

import numpy as np

import code 

##
# Pre-defined configs
##
from omni.isaac.lab_assets import RIDGEBACK_FRANKA_PANDA_CFG  # isort:skip


@configclass
class ManualBTSceneCfg(InteractiveSceneCfg):
    """Configuration for a manual created BT robot scene."""

    # warehouse environment
    environment = AssetBaseCfg(prim_path='{ENV_REGEX_NS}/Environment', 
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAACLAB_NUCLEUS_DIR}/../Environments/Simple_Warehouse/full_warehouse.usd")
    )

    # lights
    # dome_light = AssetBaseCfg(
    #     prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    # )

    # target object
    target = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Target",
        spawn=sim_utils.CuboidCfg(
            size=[0.1] * 3,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.1, 0.0), metallic=0.2)),
        init_state=RigidObjectCfg.InitialStateCfg(pos = [0.50, 0.50, 0.0]),
    )

    # articulation
    robot: ArticulationCfg = RIDGEBACK_FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    contact_forces = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger", update_period=0.0, history_length=6, debug_vis=True
    )


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0

    # Simulation loop
    while simulation_app.is_running():
        # Reset
        if count % 500 == 0:
            # Reset Counter
            count = 0

            # Reset Robot Root state
            root_state = robot.data.default_root_state.clone()
            root_state[:, :3] += scene.env_origins

            # initial position referenced by env
            root_state[:, :3] += torch.tensor([0, 0, 0.7],device=args_cli.device)

            # initial orientation
            root_state[:, 3:7] += torch.tensor([0, -np.pi/3, 0, 0],device=args_cli.device)

            robot.write_root_state_to_sim(root_state)

            # Reset Robot Joint State
            joint_pos, joint_vel = (
                robot.data.default_joint_pos.clone(),
                robot.data.default_joint_vel.clone(),
            )
            robot.write_joint_state_to_sim(joint_pos,joint_vel)

            scene.reset()
            print("[INFO]: Resetting robot state...")
        # Apply action
        # -- generate joint efforts
        # efforts = torch.randn_like(robot.data.joint_pos) * 500.0
        # ['panda_joint2', 'panda_joint3', 'panda_joint1', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'dummy_base_revolute_z_joint', 'panda_joint7', 'dummy_base_prismatic_y_joint', 'dummy_base_prismatic_x_joint', 'panda_finger_joint1', 'panda_finger_joint2']
        # efforts = torch.tensor([500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],device=args_cli.device) * args_cli.num_envs
        # -- apply action to the robot
        # robot.set_joint_effort_target(efforts)
        # -- write data to sim
        # scene.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)

        print(scene["contact_forces"])
        print("Received max contact force of: ", torch.max(scene["contact_forces"].data.net_forces_w).item())

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
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()