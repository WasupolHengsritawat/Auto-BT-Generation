import os
import torch
import queue
from datetime import datetime

# =====================================================================================================
from torch.utils.data import Dataset, DataLoader, ConcatDataset
from torch.utils.tensorboard import SummaryWriter
import numpy as np
import rclpy
import time
import yaml
import json
import pandas as pd
import numpy as np
import sys
import rclpy
import psutil
import py_trees
import networkx as nx
from scipy.spatial.transform import Rotation
from std_msgs.msg import UInt8, String
from collections import namedtuple
from itertools import groupby

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

from gymEnv import Simple_MultiBTEnv
from mcts import MCTS
from network import RvNN_mem, RvNN

from autogen_bt_interface.msg import StringStamped

from simple_bt_manager import run_simple_BTs, stop_simple_BTs
from ros2_nodes.ros2_scene_publisher import pub_scene_data

from ros2_nodes.ros2_scene_publisher import SceneNode
from ros2_nodes.ros2_battery import BatteryManager
from ros2_nodes.ros2_drive import RobotDriverManager
from ros2_nodes.ros2_bt_tracker import BTStatusTrackerNode

from env_state_machine import SearchAndDeliverMachine

from simple_behavior import (
    PatrolNode, FindTargetNode, AreObjectsExistOnInternalMap, 
    GoToNearestTarget, AreObjectNearby, PickObject, IsRobotAtTheSpawn, 
    IsObjectInHand, DropObject, GoToSpawnNode, AreXObjectsAtSpawn
)

class SituationNode:
    def __init__(self, 
                 situation,
                 mcts,
                 id,
                 acTree_node_limit=10,
                 log_dir=None,
                 ):
        """
        Docstring for __init__

        :param self: Description
        :param situation: Description
        :param mcts: Description
        :param acTree_node_limit: Description
        :param actTree_gen_epsilon: Description
        """
        # Initialize parameters
        self.situation = situation
        self.mcts = mcts
        self.acTree_node_limit = acTree_node_limit
        self.log_dir = log_dir
        self.id = id

        self.condition_tree_string = self.get_condition_tree_string(situation)
        
    def get_condition_tree_string(self, situation):
        """
        Docstring for get_condition_tree_string
        
        :param self: Description
        :param situation: Description
        """
        condition_tree_string = "(0"

        for cond_string, status in situation._asdict().items():
            if status:
                condition_tree_string += cond_string  
            else:
                condition_tree_string += f'(3{cond_string})'  

        condition_tree_string += ")"

        return condition_tree_string
    
    def get_action_tree_strings(self, n_top_action=1, n_tree_gen_iteration_per_top_action=1):
        """
        Docstring for get_action_tree_string
        
        :param n_top_action: Top n actions to consider
        :param n_tree_gen_iteration_per_top_action: Number of tree generation iterations per top action
        """
        bt_strings = []
        resulting_situations = []
        rewards = []

        # Get the action probabilities from MCTS search 
        print(f"====================================================================================================")
        print(f"[INFO] Initializing action trees for situation {self.situation}...")
        save_path = os.path.join(self.log_dir, f"situation_{self.id}", f"mcts_tree_0.json")
        action_prob = self.mcts.run_search(
                root_state='', 
                sim_initial_state_value=self.situation,
                PUCT=False, 
                temperature=1, 
                verbose=True,   # Set verbose = True if want to see each search step run time
                export_path=save_path)

        top_actions = np.argsort(action_prob)[-n_top_action:][::-1]

        tree_count = 1
        for action in top_actions:
            if action != 0:
                greedy = True
                for _ in range(n_tree_gen_iteration_per_top_action):
                    print(f"----------------------------------------------------------------------------------------------------")
                    print(f"[INFO] Creating {tree_count}/{n_tree_gen_iteration_per_top_action * n_top_action} trees for situation {self.situation}")
                    # Reset bt_string and number_of_nodes for each top action
                    number_of_nodes = 0
                    bt_string = ''
                    progress_from_current_situation = True
                    resulting_situation = self.situation
                    first_iteration = True

                    # # Modify the BT string based on top action 
                    selected_nt, selected_loc = self.decode_action(action)
                    bt_string = self.modify_bt(self.mcts.env.node_dict, bt_string, selected_nt, selected_loc)
                    number_of_nodes += 1

                    print(f"[INFO] {number_of_nodes}/{self.acTree_node_limit} nodes added. Current BT string: {bt_string}")
                
                    # The rest of the tree generation process from given bt_string
                    while True:
                        # [1] Get the action probabilities from MCTS search ==========================================
                        save_path = os.path.join(self.log_dir, f"situation_{self.id}", f"bt_{tree_count}", f"mcts_tree_{number_of_nodes}.json")
                        action_prob = self.mcts.run_search(
                            root_state=bt_string, 
                            sim_initial_state_value=self.situation,
                            PUCT=False, 
                            temperature=1, 
                            verbose=True,
                            export_path=save_path) # Set verbose = True if want to see each search step run time

                        # [2] Sample an action from searched probability =============================================
                        if greedy:
                            max_action_prob = np.max(action_prob)
                            best_action_indices = np.where(action_prob == max_action_prob)[0]

                            # Randomly select one of the best indices
                            selected_action = np.random.choice(best_action_indices)

                            greedy = False
                        else:
                            selected_action = np.random.choice(len(action_prob), p=action_prob)

                        # Decode the selected action into node type and location
                        selected_nt, selected_loc = self.decode_action(selected_action)

                        # [3] Modify the BT string based on the selected action ======================================
                        bt_string_temp = self.modify_bt(self.mcts.env.node_dict, bt_string, selected_nt, selected_loc)
                        number_of_nodes += 1

                        # Check if the situation progress if the selected node is an action node
                        if selected_nt > 3 or first_iteration:
                            self.mcts.env.set_bt(env_id=0, bt_string=f"(0{self.condition_tree_string}{bt_string_temp})")
                            _, rews, _, infos =  self.mcts.env.evaluate_bt_in_sim(sim_initial_state_value=self.situation)
                            resulting_situation_history = [key for key, group in groupby(infos[0])]
                            resulting_situation = resulting_situation_history[-1]
                            reward = rews[0]
                            first_iteration = False

                            if len(resulting_situation_history) != 1:
                                if resulting_situation_history[-2] != self.situation:
                                    progress_from_current_situation = False

                        if progress_from_current_situation:
                            bt_string = bt_string_temp
                            print(f"[INFO] {number_of_nodes}/{self.acTree_node_limit} nodes added. Current BT string: {bt_string}")
                        else:
                            print(f"[INFO] Skipping adding node to construct {bt_string_temp} due to situation jumping")

                        # Check stopping criteria
                        if selected_nt == 0 or number_of_nodes >= self.acTree_node_limit or not(progress_from_current_situation):
                            break

                    # Store the generated bt_string and resulting situation
                    if bt_string not in bt_strings:
                        bt_strings.append(bt_string)
                        resulting_situations.append(resulting_situation)
                        rewards.append(reward)

                    tree_count += 1
            
        sorted_indices = np.argsort(rewards)[::-1]
        bt_strings = [bt_strings[i] for i in sorted_indices]
        resulting_situations = [resulting_situations[i] for i in sorted_indices]
        rewards = [rewards[i] for i in sorted_indices]

        return bt_strings, resulting_situations, rewards
    
    def decode_action(self, action):
        """Decode action index into node type and location"""
        num_node_types = len(self.mcts.env.node_dict.items()) - 1
        if action > 3:
            selected_nt = (action - 4) % (num_node_types) + 1
            selected_loc = (action - 4) // (num_node_types) + 1
        else:
            selected_nt = action
            selected_loc = 0

        return selected_nt, selected_loc
    
    def modify_bt(self, node_dict, current_bt, node_type, node_location):
        """Modify the BT string"""
        node = node_dict[node_type]

        # Add a node to the BT string at the specified location
        bt_string = current_bt

        if bt_string == '':
            # If the string is empty, just return the node
            if node is None:
                return ''
            return node

        if node != None:

            # If node location is 0 and node type is a flow control node, we add it as a parent node
            if node_location == 0 and node_type in [1, 2, 3]:
                return f'({node_type - 1}' + bt_string + ')'
            
            else:
                # Iterate over all potential insertion positions (0 to len(s))
                valid_indices = [j for j in range(1,len(bt_string)) if j == len(bt_string) or not bt_string[j].isdigit()]

                if 1 <=  node_location < len(valid_indices) + 1:
                    # Get the actual index in the string where we want to insert the char.
                    insert_index = valid_indices[node_location - 1]

                    # Return the new string with the character inserted.
                    return bt_string[:insert_index] + node + bt_string[insert_index:]
            
        return current_bt

class BTGenerator:
    def __init__(
            self,
            string_condition_nodes,
            string_action_nodes,
            mcts_sim_env,
            mcts_iterations_per_search=100,
            mcts_iterations_per_eval=10,
            mcts_exploration_weight=1.0,
            mcts_allow_duplicate_nodes=True,
            mcts_fitness_mode='sparse',
            log_dir=None,
            device='cpu',
        ):
        """
        Docstring for __init__
        
        :param self: Description
        :param string_condition_nodes: Description 
        :param string_action_nodes: Description
        :param mcts_sim_env: Description
        :param mcts_iterations: Description
        :param mcts_exploration_weight: Description
        :param mcts_allow_duplicate_nodes: Description
        :param mcts_fitness_mode: Description
        :param device: Description
        """
        self.log_dir = log_dir

        # Lists of condition and action nodes in string format
        self.string_condition_nodes = string_condition_nodes
        self.string_action_nodes = string_action_nodes

        self.mcts_sim_env = mcts_sim_env

        # MCTS Initialization 
        self.mcts = MCTS(
            env = self.mcts_sim_env, 
            policy_net = None, 
            num_simulations=mcts_iterations_per_search, 
            num_eval=mcts_iterations_per_eval,
            exploration_weight=mcts_exploration_weight, 
            allow_duplicate_nodes=mcts_allow_duplicate_nodes, 
            fitness_mode=mcts_fitness_mode, 
            power_mean_constant=1.0,
            model_based=False, 
            device=device)

        # Local Parameters ==============================================
        # Named tuple for situation representation
        self.Situation = namedtuple('Situation', string_condition_nodes)

        # Default Global BT
        self.bt_string = ""
    
        # ROS2 BT Status Tracker Node
        rclpy.init()
        self.bt_tracker = BTStatusTrackerNode(num_envs=1)

    def generate(self, n_top_action = 1, n_tree_gen_iteration_per_top_action=1, ultimate_condition = None):

        if ultimate_condition is None:
            ultimate_condition = self.string_condition_nodes[-1]

        # Create a situation graph
        self.situation_graph = nx.DiGraph()

        # Get the current situation as use it as the initial situation
        id = 0
        last_situation = self.get_situation()

        buffer = []
        visited = []
        path_node = []
        path_edge = []

        # Create the situation node and add it to the graph
        last_situation_node = SituationNode(last_situation, self.mcts, id=id, log_dir=self.log_dir, acTree_node_limit=self.mcts.env.nodes_limit)
        self.situation_graph.add_node(last_situation_node)
        buffer.append(last_situation_node)
        visited.append(last_situation_node)
        path_node.append(last_situation_node)

        terminal = False

        start_time = time.time()

        while len(buffer) > 0:
            has_valid_successors_flag = False

            action_tree_strings, current_situations, rewards = last_situation_node.get_action_tree_strings(n_top_action, n_tree_gen_iteration_per_top_action)

            # Break if there exist no new situation 
            if all([current_situation == last_situation for current_situation in current_situations]):
                break
            
            # Expand the situation graph until the ultimate condition is met
            for current_situation, action_tree_string, reward in reversed(list(zip(current_situations, action_tree_strings, rewards))):
                
                # Check if the ultimate condition is met
                if getattr(current_situation, ultimate_condition):
                    terminal = True
                
                # Add the new situation node to the graph if it doesn't exist
                if current_situation not in self.situation_graph.nodes:
                    id = id + 1
                    current_situation_node = SituationNode(current_situation, self.mcts, id=id, log_dir=self.log_dir, acTree_node_limit=self.mcts.env.nodes_limit)
                    self.situation_graph.add_node(current_situation_node)

                # Add an edge from the last situation node to the current situation node
                if current_situation != last_situation:
                    self.situation_graph.add_edge(
                        last_situation_node, 
                        current_situation_node, 
                        action_tree_string=action_tree_string,
                        reward=reward)
                    
                # Add the current situation node to the buffer if it hasn't been visited
                if current_situation_node not in visited:
                    buffer.append(current_situation_node)
                    visited.append(current_situation_node)
                    has_valid_successors_flag = True

                if terminal:
                    break

            if not has_valid_successors_flag:
                path_node.pop()
                path_edge.pop()
            
            last_situation_node = buffer.pop()
            last_situation = last_situation_node.situation

            path_node.append(last_situation_node)
            path_edge.append(self.situation_graph.edges[path_node[-2], path_node[-1]] if len(path_node) > 1 else None)

            if terminal:
                path_node.pop()
                break

        result_tree = ""
        if terminal:
            for node, edge in zip(path_node, path_edge):
                situation_tree_string = f"(0{node.condition_tree_string}{edge['action_tree_string']})"
                result_tree = f"{result_tree}{situation_tree_string}"

            result_tree = f"(1{result_tree})"

            print(f"[INFO] Successfully generated a BT: {result_tree}")
        else:
            print("[WARNING] Unable to generate a BT that satisfies the ultimate condition within the node limit.")

        print(f"[INFO] Total generation time: {time.time() - start_time:.2f} seconds")

        return result_tree

    def get_situation(self):
        """
        Docstring for get_situation
        
        :param self: Description
        """
        situation = {}

        for string_node in self.string_condition_nodes:
            # Create the behavior tree for the condition node and run it until SUCCESS or FAILURE
            run_simple_BTs(string_node)

            while  not (self.bt_tracker.get_status(env_id=0) in ['SUCCESS', 'FAILURE']):
                # Spin BT Tracker Node
                try:
                    rclpy.spin_once(self.bt_tracker, timeout_sec=0.0)
                except rclpy.executors.ExternalShutdownException:
                    print("[INFO] Spin exited because ROS2 shutdown detected.")

            stop_simple_BTs()
        
            # After stopping subprocesses, check if the parent's ROS context is still okay
            if not rclpy.ok():
                print("[WARNING] ROS context was invalidated during stop. Re-initializing...")
                rclpy.init()

            # Record the status of the condition node
            status = self.bt_tracker.get_status(env_id=0)
            self.bt_tracker.reset_status(env_id=0)

            if status == "SUCCESS":
                situation[string_node] = True
            elif status == "FAILURE":
                situation[string_node] = False
            else:
                raise ValueError("[Error] Condition Node returned invalid status")       

        return self.Situation(**situation)

    

