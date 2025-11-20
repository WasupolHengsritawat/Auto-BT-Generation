import os
import json
import torch
import numpy as np
import sys
import os
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import UInt8, String
import py_trees
from datetime import datetime

from autogen_bt_interface.msg import StringStamped

import py_trees

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

from bt.simple_behavior import (
    PatrolNode, FindTargetNode, AreObjectsExistOnInternalMap, 
    GoToNearestTarget, AreObjectNearby, PickObject, IsRobotAtTheSpawn, 
    IsObjectInHand, DropObject, GoToSpawnNode, AreXObjectsAtSpawn
)

from simulation.env_state_machine import SearchAndDeliverMachine

# File path setup
script_dir = os.path.dirname(os.path.abspath(__file__))
logs_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))

date_time = "2025-11-07_23-21-00-fibo0-seed1-mcgs-uct-8000searchiters"
model_name = "rvnn_iter000"

loop_allowed = 2

reward_weight = [  0.025,     # Is object found
                    0.050,     # Was robot been to object
                    0.075,     # Is object picked
                    0.100,     # Was robot been to final
                    0.200,     # Is object delivered
                    -0.00025]  # Tree complexity penalty term

# Create log directory if it doesn't exist
os.makedirs(logs_dir, exist_ok=True)

class TaskState:
    def __init__(self, position: str, object_found: bool, object_picked: bool, object_delivered: bool):
        self.position = position
        self.object_found = object_found
        self.object_picked = object_picked
        self.object_delivered = object_delivered

    def _is_object_found(self): return self.object_found
    def _is_robot_at_object(self): return self.position == 'Object' or self.position == 'Final'
    def _is_object_picked(self): return self.object_picked
    def _is_robot_at_final(self): return self.position == 'Final'
    def _is_object_delivered(self): return self.object_delivered

class TaskStateProgress:
    def __init__(self, position: str, object_found: bool, object_picked: bool, object_delivered: bool):
        self.position = position
        self.object_found = object_found
        self.object_picked = object_picked
        self.object_delivered = object_delivered

    def _was_object_found(self): return self.object_found
    def _was_robot_been_to_object(self): return self.position == 'Object' or self.position == 'Final'
    def _was_object_picked(self): return self.object_picked
    def _was_robot_been_to_final(self): return self.position == 'Final'
    def _was_object_delivered(self): return self.object_delivered

def get_reward(env_state):
    state_progress = {
            'A' : TaskStateProgress(position='Start' , object_found=False, object_picked=False, object_delivered=False), # Initial state
            'B' : TaskStateProgress(position='InMap' , object_found=False, object_picked=False, object_delivered=False), # Patroling
            'C' : TaskStateProgress(position='InMap' , object_found=True , object_picked=False, object_delivered=False), # Searching -> Object found
            'D' : TaskStateProgress(position='Object', object_found=True , object_picked=False, object_delivered=False), # Arrived at object location
            'E' : TaskStateProgress(position='Object', object_found=True , object_picked=True , object_delivered=False), # Object Picked
            'F' : TaskStateProgress(position='Final' , object_found=True , object_picked=True , object_delivered=False), # Initial State with object in hand
            'G' : TaskStateProgress(position='Final' , object_found=True , object_picked=False, object_delivered=False), # Initial State with known object location
            'H' : TaskStateProgress(position='Final' , object_found=True , object_picked=True , object_delivered=True)   # Final State
        }

    rewards = [ reward_weight[0] * state_progress[final_state]._was_object_found() +
                reward_weight[1] * state_progress[final_state]._was_robot_been_to_object() +
                reward_weight[2] * state_progress[final_state]._was_object_picked() +
                reward_weight[3] * state_progress[final_state]._was_robot_been_to_final() +
                reward_weight[4] * state_progress[final_state]._was_object_delivered() for final_state in env_state ]

    return rewards

def subtract_prefix(s: str, prefix: str) -> str:
    if s.startswith(prefix):
        return s[len(prefix):]
    return s  # if prefix doesn't match, return original string

def create_tree(env_id, tree_string, verbose = False):
    """
    Create the behavior tree after obtaining the correct environment origin.
    
    Args:
        env_id (int): Environment identifier.
        node (EnvironmentSubscriber): ROS2 node that subscribes to the environment origin.
    
    Returns:
        py_trees.composites.Selector: Root node of the behavior tree.
    """

                    # Behaviors
    behavior_dict = {'a': lambda ind: PatrolNode(name = f"PatrolNode_{ind}", env_id = env_id, verbose=verbose),
                     'b': lambda ind: FindTargetNode(name=f'FindTarget_{ind}', env_id=env_id, verbose=verbose),
                     'c': lambda ind: GoToNearestTarget(name=f"GoToNearestTarget_{ind}", env_id=env_id, verbose=verbose),
                     'e': lambda ind: GoToSpawnNode(name=f'GoToSpawnNode_{ind}', env_id=env_id, verbose=verbose),
                     'f': lambda ind: PickObject(name = f'PickObject_{ind}', env_id=env_id, verbose=verbose),
                     'g': lambda ind: DropObject(name=f'DropObject_{ind}', env_id=env_id, verbose=verbose),
                    # Conditions
                     'B': lambda ind: IsRobotAtTheSpawn(name=f'IsRobotAtTheSpawn_{ind}', env_id=env_id, verbose=verbose),
                     'D': lambda ind: AreObjectsExistOnInternalMap(name=f'AreObjectExistsOnInternalMap_{ind}', env_id=env_id, verbose=verbose),
                     'E': lambda ind: AreObjectNearby(f'AreObjectNearby_{ind}', env_id=env_id, verbose=verbose),
                     'F': lambda ind: IsObjectInHand(f'IsObjectInHand_{ind}', env_id=env_id, verbose=verbose),
                     'H': lambda ind: AreXObjectsAtSpawn(f'AreFiveObjectsAtSpawn_{ind}', env_id=env_id, verbose=verbose),
                     }

    def string2tree(tree_string, cond_num):
        # If a single behavior node is passed, return the corresponding behavior node
        if len(tree_string) == 1:
            return behavior_dict[tree_string[0]](0)
        
        # Select Condition Node as Parent Node
        condition_node = tree_string[1]
        if condition_node == '0':
            parent = py_trees.composites.Sequence(f"Sequence_{cond_num}", memory=False)
        elif condition_node == '1':
            parent = py_trees.composites.Selector(f"Selector_{cond_num}", memory=False)
        elif condition_node == '2':
            parent = py_trees.composites.Parallel(f"Parallel_{cond_num}", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    
        cond_num += 1
        record = False

        child_num = 0
        for n in tree_string[2:]:
            if record:
                subtree_string += n
                if n == '(':
                    n_open += 1
                elif n == ')':
                    n_open -= 1

                if n_open == 0:
                    record = False
                    parent.add_child(string2tree(subtree_string, cond_num))
            else:
                if n == '(':
                    subtree_string = '('
                    n_open = 1
                    record = True
                elif n == ')':
                    pass
                else:
                    if n in behavior_dict.keys():
                        parent.add_child(behavior_dict[n](child_num))
                        child_num += 1
                    else:
                        print('[Error] Undefined char in tree_string')

        return parent
    
    if tree_string=='':
        return None
    
    return string2tree(tree_string, 0)

if __name__ == '__main__':
    count = 1

    while True:
        try:

            # Create a timestamped log file
            timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            log_file_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}_verification.txt")

            # Redirect stdout and stderr
            sys.stdout = open(log_file_path, "w")
            sys.stderr = sys.stdout

            print(f"[INFO] Logging started at {timestamp}")
            print(f"[INFO] Log file path: {log_file_path}")

            json_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}.json")
            output_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}.html")

            # Load JSON data
            with open(json_path, "r") as f:
                data = json.load(f)

            node_list = []
            for node in data["nodes"]:
                node_list.append(node)

            node_list.pop(0)

            state_value_pair = [(node["state"], node["value"]) for node in node_list if node["state"] != '']
            bt_string_list, bt_value_list = list(zip(*state_value_pair))
            num_envs = len(state_value_pair)

            # BT blackboard Initialization
            bb_client = py_trees.blackboard.Client(name="External")

            # Behavior Tree Setup
            trees = []
            for env_id in range(num_envs):
                # BT initialization
                root = create_tree(env_id, bt_string_list[env_id], verbose=False)
                tree = py_trees.trees.BehaviourTree(root)
                tree.setup(timeout=15)
                trees.append(tree)

                # BT blackboard variable registration
                bb_client.register_key(key=f"action_{env_id}", access=py_trees.common.Access.READ)
                bb_client.register_key(key=f"action_{env_id}", access=py_trees.common.Access.WRITE)

                bb_client.register_key(key=f"env_state_{env_id}", access=py_trees.common.Access.WRITE)

                bb_client.set(f"action_{env_id}", '')

            # Environment Finite State Machine Setup
            env_fsm = [SearchAndDeliverMachine() for _ in range(num_envs)]
            env_state = [fsm.current_state.id for fsm in env_fsm]
            env_state_history = [fsm.current_state.id for fsm in env_fsm]
            env_done = [False for _ in range(num_envs)]
            bb_shared_data_last = ["" for _ in range(num_envs)]

            # print(f"[INFO] Initial State: {env_state}")

            while not all(env_done):  
                # print("\n[External] State:", env_state)

                # Update state in BT blackboard
                for env_id in range(num_envs):
                    # Update the environment state in the blackboard
                    bb_client.set(f"env_state_{env_id}", env_state[env_id])

                    # Execute a BT tick
                    trees[env_id].tick()

                    # Update the action in the blackboard
                    bb_client.set(f"action_{env_id}", subtract_prefix(bb_client.get(f"action_{env_id}"), bb_shared_data_last[env_id]))
                    bb_shared_data_last[env_id] = bb_client.get(f"action_{env_id}")

                    # print(f"[External] BT Action {env_id}: {bb_client.get(f'action_{env_id}')}")

                    try:
                        # Send the action to the environment FSM
                        env_fsm[env_id].send(bb_client.get(f"action_{env_id}"))
                        env_state[env_id] = env_fsm[env_id].current_state.id

                        # Stop the individual simulation if the FSM reached the accepted state
                        if env_state[env_id] == 'H':
                            env_done[env_id] = True
                    except Exception as e:
                        # Stop the individual simulation if the FSM rejects the command
                        if bb_client.get(f'action_{env_id}') != "":
                            env_done[env_id] = True

                    # Stop the individual simulation if the FSM reached the maximum number of loops
                    env_state_history[env_id] += env_state[env_id]
                    if env_state_history[env_id].count(env_state[env_id]) >= (loop_allowed + 1):
                        env_done[env_id] = True

            print("\n*************** Pass ***************")
            # print("\n[External] Final State:", env_state)

            final_rewards = get_reward(env_state)

            expected_rewards = []
            for state in env_state:
                if state == 'C':
                    expected_rewards.append(0.025)
                elif state == 'D':
                    expected_rewards.append(0.075)
                else:
                    expected_rewards.append(0.0)

            match_count_actual = 0
            expected_match_count = 0

            recorded_mismatch = []
            calculation_mismatch = []
            flag = False

            for bt_string, env_state, expected_reward, actual_reward, value in zip(bt_string_list, env_state, expected_rewards, final_rewards, bt_value_list):
                
                if abs(actual_reward - value) < 1e-5:
                    match_count_actual += 1
                else:
                    flag = True

                if abs(expected_reward - value) < 1e-5:
                    expected_match_count += 1
                else:
                    flag = True

                if flag:
                    recorded_mismatch.append((bt_string, env_state, expected_reward, actual_reward, value))
                    flag = False

                if abs(expected_reward - actual_reward) > 1e-5:
                    calculation_mismatch.append((bt_string, env_state, expected_reward, actual_reward, value))

            print(f"[INFO] Matching rewards: {match_count_actual}/{num_envs} -> {match_count_actual / num_envs:.2%} (with actual reward calculations)")
            print(f"[INFO] Expected matches: {expected_match_count}/{num_envs} -> {expected_match_count / num_envs:.2%} (with expected reward calculations)")

            if recorded_mismatch:
                print("\n[WARNING] Mismatched BT strings and rewards:")
                for i, (bt_string, env_state, expected_reward, actual_reward, value) in enumerate(recorded_mismatch):
                    print(f"  [{i:4d}] BT String: {bt_string:<30} | Final State: {env_state:<8} | Expected Reward: {expected_reward:8.3f} | Actual Reward: {actual_reward:8.3f} | Recorded Value: {value:8.3f}")

            if calculation_mismatch:
                print("\n[WARNING] Calculation mismatched BT strings and rewards:")
                for i, (bt_string, env_state, expected_reward, actual_reward, value) in enumerate(calculation_mismatch):
                    print(f"  [{i:4d}] BT String: {bt_string:<30} | Final State: {env_state:<8} | Expected Reward: {expected_reward:8.3f} | Actual Reward: {actual_reward:8.3f} | Recorded Value: {value:8.3f}")
        
            count += 1
            
        except:
            break
