###
#  Simulation Configuration
###
import torch
import numpy as np
import time
import sys
import os
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import UInt8, String

from autogen_bt_interface.msg import StringStamped

# Import local files
# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
# simulation_dir = os.path.abspath(os.path.join(project_root, "scripts", "simulation"))
bt_dir = os.path.abspath(os.path.join(project_root, "scripts", "bt"))

# Add it to sys.path 
sys.path.insert(0, project_root)
sys.path.insert(0, script_dir)
# sys.path.insert(0, simulation_dir)
sys.path.insert(0, bt_dir)

from simple_bt_manager import run_simple_BTs, stop_simple_BTs

from env_state_machine import SearchAndDeliverMachine

######## Hyperparameters ########
num_envs = 1
#################################

class TaskState:
    def __init__(self, position: str, object_found: bool, object_picked: bool, object_delivered: bool):
        self.position = position
        self.object_found = object_found
        self.object_picked = object_picked
        self.object_delivered = object_delivered

    def _is_object_found(self): return self.object_found
    def _was_robot_been_to_object(self): return self.position == 'Object' or self.position == 'Final'
    def _is_object_picked(self): return self.object_picked
    def _was_robot_been_to_final(self): return self.position == 'Final'
    def _is_object_delivered(self): return self.object_delivered

def subtract_prefix(s: str, prefix: str) -> str:
    if s.startswith(prefix):
        return s[len(prefix):]
    return s  # if prefix doesn't match, return original string

def _receive_action(env_id, msg):
    """
    Update a specific environment's internal state synced with ROS2 topics.

    :param idx: Index of the environment to update.
    :param attr: Name of the attribute to set.
    :param value: New value received from ROS2.
    """
    # Recieved action and timestamp
    current_data = msg.data
    robot_action[env_id] = robot_action[env_id] + current_data
    # print(f"[INFO] Received action '{current_data}' for environment {env_id}")

    tick_flag[env_id] = True

if __name__ == '__main__':
    if not rclpy.ok():
        rclpy.init()

    node = Node('multi_bt_env')
    env_publisher = []

    for i in range(num_envs):
        env_publisher.append(node.create_publisher(String, f'/env_{i}/robot/state', 10))
        node.create_subscription(StringStamped, f'/env_{i}/robot/action', lambda msg, idx=i: _receive_action(idx, msg), 10)

    # State Progress for Reward Calculation
    state_progress = {
        'A' : TaskState(position='Start' , object_found=False, object_picked=False, object_delivered=False), # Initial state
        'B' : TaskState(position='InMap' , object_found=False, object_picked=False, object_delivered=False), # Patroling
        'C' : TaskState(position='InMap' , object_found=True , object_picked=False, object_delivered=False), # Searching -> Object found
        'D' : TaskState(position='Object', object_found=True , object_picked=False, object_delivered=False), # Arrived at object location
        'E' : TaskState(position='Object', object_found=True , object_picked=True , object_delivered=False), # Object Picked
        'F' : TaskState(position='Final' , object_found=True , object_picked=True , object_delivered=False), # Initial State with object in hand
        'G' : TaskState(position='Final' , object_found=True , object_picked=False, object_delivered=False), # Initial State with known object location
        'H' : TaskState(position='Final' , object_found=True , object_picked=True , object_delivered=True)   # Final State
    }

    # bt_string_array = ['(1H(0(1F(0(1E(0(1D(2ab))c))f))(1Be)g))'] * num_envs
    # bt_string_array = ['(1E(0(1D(2ab))c)'] * num_envs

    # run_simple_BTs(bt_string_array) # run BTs

    # Environment Finite State Machine Setup
    env_fsm = [SearchAndDeliverMachine() for _ in range(num_envs)]
    env_state = [fsm.current_state.id for fsm in env_fsm]
    env_done = [False for _ in range(num_envs)]

    # Simulation Variables
    robot_action = ["" for _ in range(num_envs)]
    last_timestamp = [0 for _ in range(num_envs)]
    tick_flag = [False for _ in range(num_envs)]
    last_tick_flag = [False for _ in range(num_envs)]

    print(f"[INFO] Initial State: {env_state}")

    while all(env_done) != True:

        # Spin ROS2 node to receive messages
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except rclpy.executors.ExternalShutdownException:
            print("[INFO] Spin exited because ROS2 shutdown detected.")

        # Publish current environment state
        for i in range(num_envs):
             env_publisher[i].publish(String(data=env_state[i]))

        # If any robot action is None, skip the step
        # print("[DEBUG] Robot Actions:", robot_action)
        if "" in robot_action:
            continue

        for i in range(num_envs):
            # Check if the BT has just ticked
            if (not last_tick_flag[i]) and tick_flag[i]:
                last_timestamp[i] = time.time()
                last_tick_flag[i] = True

            # At the end of the BT tick, do the following
            if time.time() - last_timestamp[i] > 1e-3 and tick_flag[i]:

                # [1] Send the action to the environment FSM to get the next state
                try:
                    print(f"[INFO] Sending action {robot_action[i]} to environment {i} in state {env_state[i]}")
                    env_fsm[i].send(robot_action[i])
                    env_state[i] = env_fsm[i].current_state.id

                    if env_state[i] == 'H':
                        env_done[i] = True
                except:
                    env_done[i] = True

                # [2] Reset for the next BT tick
                tick_flag[i] = False
                last_tick_flag[i] = False
                robot_action[i] = ""

                print(f"[INFO] Resulting state: {env_state}")

    print(f"[INFO] Final State: {env_state}")
    print("[INFO] Finished simulation execution.")
    node.destroy_node()
    # stop_simple_BTs(verbose=True)