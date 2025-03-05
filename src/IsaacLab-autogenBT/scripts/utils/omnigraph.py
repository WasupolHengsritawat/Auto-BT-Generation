import omni
import omni.graph.core as og


def create_controller_omnigraph(env_ind):

    # Path to Environment
    environment_path = f"/World/envs/env_{env_ind}"

    # Create Omnigraph to use as an Interface between ROS2 and the Simulation
    try:
        og.Controller.edit(
            {"graph_path": f"/World/ActionGraph_{env_ind}", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    # ROS2 and Simulation Context
                    ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("SimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),

                    # Sensors =================================================================================
                    # # Lidar
                    # ("OneSimulationFrame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),
                    # ("CreateRenderProduct", "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                    # ("LidarHelperScan", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),

                    # Contact
                    ("ReadContact_R", "omni.isaac.sensor.IsaacReadContactSensor"),
                    ("ReadContact_L", "omni.isaac.sensor.IsaacReadContactSensor"),
                    ("ContactPublish_R", "omni.isaac.ros2_bridge.ROS2Publisher"),
                    ("ContactPublish_L", "omni.isaac.ros2_bridge.ROS2Publisher"),

                    # Controllers =============================================================================
                    # Four-wheel Mobile Robot Controller
                    ("CmdVelSubscribe", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                    ("ScaleToFromStageUnit", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                    ("BreakLinearCmdVel", "omni.graph.nodes.BreakVector3"),
                    ("BreakAngularCmdVel", "omni.graph.nodes.BreakVector3"),
                    ("DifferentialController", "omni.isaac.wheeled_robots.DifferentialController"),
                    ("GetCmdVelInd0", "omni.graph.nodes.ArrayIndex"),
                    ("GetCmdVelInd1", "omni.graph.nodes.ArrayIndex"),
                    ("CmdVelInd0Array", "omni.graph.nodes.ConstructArray"),
                    ("CmdVelInd1Array", "omni.graph.nodes.ConstructArray"),
                    ("CmdVelLeftArray", "omni.graph.nodes.AppendArray"),
                    ("CmdVelRightArray", "omni.graph.nodes.AppendArray"),
                    ("CmdVelArray", "omni.graph.nodes.AppendArray"),
                    ("JointName1", "omni.graph.nodes.ConstantToken"),
                    ("JointName2", "omni.graph.nodes.ConstantToken"),
                    ("JointName3", "omni.graph.nodes.ConstantToken"),
                    ("JointName4", "omni.graph.nodes.ConstantToken"),
                    ("Joint1Array", "omni.graph.nodes.ConstructArray"),
                    ("Joint2Array", "omni.graph.nodes.ConstructArray"),
                    ("Joint3Array", "omni.graph.nodes.ConstructArray"),
                    ("Joint4Array", "omni.graph.nodes.ConstructArray"),
                    ("JointLeftArray", "omni.graph.nodes.AppendArray"),
                    ("JointRightArray", "omni.graph.nodes.AppendArray"),
                    ("JointNameArray", "omni.graph.nodes.AppendArray"),
                    ("MobileArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),

                    # Manipulator Controller
                    ("JointStateSubscribe", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
                    ("ManiArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),

                    # =========================================================================================
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Sensors =================================================================================
                    # Lidar
                    # ("CreateRenderProduct.inputs:cameraPrim", f"{environment_path}/Robot/Jackal/base_link/sick_lms1xx_lidar_frame/lidar"),
                    # ("LidarHelperScan.inputs:topicName", f"/robot_{env_ind}/laser_scan"),
                    # ("LidarHelperScan.inputs:type", "laser_scan"),

                    # Contact
                    ("ReadContact_R.inputs:csPrim", f"{environment_path}/Robot/UR5/Robotiq_Hand_E/right_gripper/Contact_Sensor"),
                    ("ReadContact_L.inputs:csPrim", f"{environment_path}/Robot/UR5/Robotiq_Hand_E/left_gripper/Contact_Sensor"),
                    ("ContactPublish_R.inputs:messageName", "Float64"),
                    ("ContactPublish_R.inputs:messagePackage", "std_msgs"),
                    ("ContactPublish_R.inputs:messageSubfolder", "msg"),
                    ("ContactPublish_R.inputs:topicName", f"/env_{env_ind}/robot/eef_contact_r"),
                    ("ContactPublish_L.inputs:messageName", "Float64"),
                    ("ContactPublish_L.inputs:messagePackage", "std_msgs"),
                    ("ContactPublish_L.inputs:messageSubfolder", "msg"),
                    ("ContactPublish_L.inputs:topicName", f"/env_{env_ind}/robot/eef_contact_l"),

                    # Controllers =============================================================================
                    # Four-wheel Mobile Robot Controller
                    ("CmdVelSubscribe.inputs:topicName", f"/env_{env_ind}/robot/cmd_vel"),
                    ("DifferentialController.inputs:maxWheelSpeed", 500.0),
                    ("DifferentialController.inputs:wheelDistance", 0.37558),
                    ("DifferentialController.inputs:wheelRadius", 0.098),
                    ("GetCmdVelInd0.inputs:index", 0),
                    ("GetCmdVelInd1.inputs:index", 1),
                    ("JointName1.inputs:value", "front_left_wheel_joint"),
                    ("JointName2.inputs:value", "rear_left_wheel_joint"),
                    ("JointName3.inputs:value", "front_right_wheel_joint"),
                    ("JointName4.inputs:value", "rear_right_wheel_joint"),
                    ("MobileArticulationController.inputs:targetPrim", f"{environment_path}/Robot"),

                    # Manipulator Controller
                    ("JointStateSubscribe.inputs:topicName", f"/env_{env_ind}/robot/joint_command"),
                    ("ManiArticulationController.inputs:targetPrim", f"{environment_path}/Robot"),

                    # =========================================================================================
                ],
                og.Controller.Keys.CONNECT: [
                    # ROS2 and Simulation Context
                    ("Context.outputs:context", "CmdVelSubscribe.inputs:context"),
                    ("Context.outputs:context", "ContactPublish_R.inputs:context"),
                    ("Context.outputs:context", "ContactPublish_L.inputs:context"),
                    ("OnPlaybackTick.outputs:tick", "CmdVelSubscribe.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "MobileArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "JointStateSubscribe.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ManiArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ReadContact_R.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ReadContact_L.inputs:execIn"),

                    # Sensors =================================================================================
                    # Lidar
                    # ("Context.outputs:context", "LidarHelperScan.inputs:context"),
                    # ("OnPlaybackTick.outputs:tick", "OneSimulationFrame.inputs:execIn"),
                    # ("OneSimulationFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),
                    # ("CreateRenderProduct.outputs:execOut", "LidarHelperScan.inputs:execIn"),
                    # ("CreateRenderProduct.outputs:renderProductPath", "LidarHelperScan.inputs:renderProductPath"),

                    # Contact
                    ("ReadContact_R.outputs:execOut", "ContactPublish_R.inputs:execIn"),
                    # ("ReadContact_R.outputs:value", "ContactPublish_R.inputs:data"),
                    ("ReadContact_L.outputs:execOut", "ContactPublish_L.inputs:execIn"),
                    # ("ReadContact_L.outputs:value", "ContactPublish_L.inputs:data"),

                    # Controllers =============================================================================
                    # Four-wheel Mobile Robot Controller
                    ("CmdVelSubscribe.outputs:execOut", "DifferentialController.inputs:execIn"),
                    ("CmdVelSubscribe.outputs:angularVelocity", "BreakAngularCmdVel.inputs:tuple"),
                    ("CmdVelSubscribe.outputs:linearVelocity", "ScaleToFromStageUnit.inputs:value"),
                    ("ScaleToFromStageUnit.outputs:result", "BreakLinearCmdVel.inputs:tuple"),
                    ("BreakAngularCmdVel.outputs:z", "DifferentialController.inputs:angularVelocity"),
                    ("BreakLinearCmdVel.outputs:x", "DifferentialController.inputs:linearVelocity"),
                    ("DifferentialController.outputs:velocityCommand", "GetCmdVelInd0.inputs:array"),
                    ("DifferentialController.outputs:velocityCommand", "GetCmdVelInd1.inputs:array"),
                    ("GetCmdVelInd0.outputs:value", "CmdVelInd0Array.inputs:input0"),
                    ("CmdVelInd0Array.outputs:array", "CmdVelLeftArray.inputs:input0"),
                    ("CmdVelInd0Array.outputs:array", "CmdVelLeftArray.inputs:input1"),
                    ("GetCmdVelInd1.outputs:value", "CmdVelInd1Array.inputs:input0"),
                    ("CmdVelInd1Array.outputs:array", "CmdVelRightArray.inputs:input0"),
                    ("CmdVelInd1Array.outputs:array", "CmdVelRightArray.inputs:input1"),
                    ("CmdVelLeftArray.outputs:array", "CmdVelArray.inputs:input0"),
                    ("CmdVelRightArray.outputs:array", "CmdVelArray.inputs:input1"),
                    ("CmdVelArray.outputs:array", "MobileArticulationController.inputs:velocityCommand"),
                    ("JointName1.inputs:value", "Joint1Array.inputs:input0"),
                    ("JointName2.inputs:value", "Joint2Array.inputs:input0"),
                    ("JointName3.inputs:value", "Joint3Array.inputs:input0"),
                    ("JointName4.inputs:value", "Joint4Array.inputs:input0"),
                    ("Joint1Array.outputs:array", "JointLeftArray.inputs:input0"),
                    ("Joint2Array.outputs:array", "JointLeftArray.inputs:input1"),
                    ("Joint3Array.outputs:array", "JointRightArray.inputs:input0"),
                    ("Joint4Array.outputs:array", "JointRightArray.inputs:input1"),
                    ("JointLeftArray.outputs:array", "JointNameArray.inputs:input0"),
                    ("JointRightArray.outputs:array", "JointNameArray.inputs:input1"),
                    ("JointNameArray.outputs:array", "MobileArticulationController.inputs:jointNames"),

                    # Manipulator Controller
                    ("JointStateSubscribe.outputs:effortCommand", "ManiArticulationController.inputs:effortCommand"),
                    ("JointStateSubscribe.outputs:velocityCommand", "ManiArticulationController.inputs:velocityCommand"),
                    ("JointStateSubscribe.outputs:positionCommand", "ManiArticulationController.inputs:positionCommand"),
                    ("JointStateSubscribe.outputs:jointNames", "ManiArticulationController.inputs:jointNames"),

                    # =========================================================================================
                ],
            },
        )

        og.Controller.connect(f"/World/ActionGraph_{env_ind}/ReadContact_R.outputs:value", f"/World/ActionGraph_{env_ind}/ContactPublish_R.inputs:data")
        og.Controller.connect(f"/World/ActionGraph_{env_ind}/ReadContact_L.outputs:value", f"/World/ActionGraph_{env_ind}/ContactPublish_L.inputs:data")
        
    except Exception as e:
        print(e)