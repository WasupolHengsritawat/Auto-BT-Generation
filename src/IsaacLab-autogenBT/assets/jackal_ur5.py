# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Ridgeback-Manipulation robots.

The following configurations are available:

* :obj:`RIDGEBACK_FRANKA_PANDA_CFG`: Clearpath Ridgeback base with Franka Emika arm

Reference: https://github.com/ridgeback/ridgeback_manipulation
"""

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

import os 
import code 

##
# Configuration
##

# Get current file path
script_dir = os.path.dirname(os.path.abspath(__file__))

JACKAL_UR5_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{script_dir}/jackal_ur5.usd",
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=False),
        activate_contact_sensors=True,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # base
            "front_left_wheel_joint": 0.0,
            "front_right_wheel_joint": 0.0,
            "rear_left_wheel_joint": 0.0,
            "rear_right_wheel_joint": 0.0,
            # franka arm
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.047,
            "elbow_joint": 1.833,
            "wrist_1_joint": -0.785,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
            # tool
            "Slider_.*": 0.025,
        },
        # joint_pos={
        #     # base
        #     "front_left_wheel_joint": 0.0,
        #     "front_right_wheel_joint": 0.0,
        #     "rear_left_wheel_joint": 0.0,
        #     "rear_right_wheel_joint": 0.0,
        #     # franka arm
        #     "shoulder_pan_joint": 0.0,
        #     "shoulder_lift_joint": 0.0,
        #     "elbow_joint": 0.0,
        #     "wrist_1_joint": 0.0,
        #     "wrist_2_joint": 0.0,
        #     "wrist_3_joint": 0.0,
        #     # tool
        #     "Slider_.*": 0.025,
        # },
        joint_vel={".*": 0.0},
        pos=[0,0,0],
        rot=[1,0,0,0],
    ),
    actuators={
        "mobile": ImplicitActuatorCfg(
            joint_names_expr=["front_left_wheel_joint","front_right_wheel_joint","rear_left_wheel_joint","rear_right_wheel_joint"],
            velocity_limit=100.0,
            effort_limit=1000.0,
            stiffness=0.0,
            damping=1e7,
        ),
        "manipulator": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint","Slider_1","Slider_2"],
            velocity_limit=100.0,
            effort_limit=9000.0,
            stiffness=261.79941,
            damping=26.17994,
        ),
    },
)
"""Configuration of Franka arm with Franka Hand on a Clearpath Ridgeback base using implicit actuator models.

The following control configuration is used:

* Base: velocity control
* Arm: position control with damping
* Hand: position control with damping

"""
