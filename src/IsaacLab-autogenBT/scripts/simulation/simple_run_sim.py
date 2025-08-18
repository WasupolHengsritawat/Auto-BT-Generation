###
#  Simulation Configuration
###
import torch
import numpy as np
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
num_envs = 20
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

def _receive_action(env_id, msg):
    """
    Update a specific environment's internal state synced with ROS2 topics.

    :param idx: Index of the environment to update.
    :param attr: Name of the attribute to set.
    :param value: New value received from ROS2.
    """
    unparallelable_action = {'a': ['c', 'e'], 
                             'b': [],
                             'c': ['a', 'e'],
                             'e': ['a', 'c'],
                             'f': ['g'],
                             'g': ['f'],}
    temp_robot_action = msg.data
    temp_robot_action_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    if temp_robot_action_timestamp - robot_action_timestamp[env_id] < 7e-3 and temp_robot_action not in robot_action[env_id] and not(set(robot_action) & set(unparallelable_action[temp_robot_action])):
        temp_robot_action = robot_action[env_id] + temp_robot_action

    robot_action[env_id] = temp_robot_action
    robot_action_timestamp[env_id] = temp_robot_action_timestamp

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
    bt_string_array = ['(2abce)'] * num_envs

    run_simple_BTs(bt_string_array) # run BTs

    # Environment Finite State Machine Setup
    env_fsm = [SearchAndDeliverMachine() for _ in range(num_envs)]
    env_state = [fsm.current_state.id for fsm in env_fsm]
    env_done = [False for _ in range(num_envs)]

    # Simulation Variables
    robot_action = [None for _ in range(num_envs)]
    robot_action_timestamp = [0 for _ in range(num_envs)]

    print(f"[INFO] Initial State: {env_state}")

    while all(env_done) != True:
        # Spin Env Node
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except rclpy.executors.ExternalShutdownException:
            print("[INFO] Spin exited because ROS2 shutdown detected.")

        for i in range(num_envs):
            env_publisher[i].publish(String(data=env_state[i]))

        # If any robot action is None, skip the step
        if None in robot_action:
            continue

        for i in range(num_envs):
            try:
                env_fsm[i].send(robot_action[i])
                env_state[i] = env_fsm[i].current_state.id

                if env_state[i] == 'H':
                    env_done[i] = True
            except:
                env_done[i] = True
        
        print(f"[INFO] {env_state}")

    print("[INFO] Finished simulation execution.")