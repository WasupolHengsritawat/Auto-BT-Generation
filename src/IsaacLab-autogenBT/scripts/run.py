import os
import torch
import numpy as np
import rclpy
from datetime import datetime

# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, ".."))
simulation_dir = os.path.abspath(os.path.join(project_root, "scripts", "simulation"))
bt_dir = os.path.abspath(os.path.join(project_root, "scripts", "bt"))

from learning.BTGenerator import BTGenerator
from learning.gymEnv import Simple_MultiBTEnv

# SEED = 1
# torch.manual_seed(SEED)
# np.random.seed(SEED)

log_dir = os.path.join(project_root, "logs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

node_dict = { 
    # Special
    0: None,  
    # Flow Controls 
    1 : '(0)', #sequence_node
    2 : '(1)', #fallback_node
    3 : '(2)', #parallel_node
    # Behaviors
    4 : 'a', #patrol_node
    5 : 'b', #find_target_node
    6 : 'c', #go_to_nearest_target
    7 : 'e', #go_to_spawn_node
    8 : 'f', #picking_object_node
    9 : 'g', #drop_object_node
}

string_condition_nodes = ['B', 'D', 'E', 'F', 'H']
string_action_nodes = ['a', 'b', 'c', 'e', 'f', 'g']

env = Simple_MultiBTEnv(
    node_dict, 
    nodes_limit=5, 
    num_envs=8,
    loop_allowed=2,
    verbose=False)

bt_generator = BTGenerator(
    string_condition_nodes=string_condition_nodes,
    string_action_nodes=string_action_nodes,
    mcts_sim_env=env,
    mcts_iterations_per_search=200,
    mcts_iterations_per_eval=10,
    mcts_exploration_weight=1.0,
    mcts_allow_duplicate_nodes=True,
    mcts_fitness_mode=None,
    log_dir=log_dir,
    device='cpu',
)


situation = bt_generator.get_situation()

print("Current Situation:", situation)

result = bt_generator.generate(
    ultimate_condition='H', 
    n_top_action=1, 
    n_tree_gen_iteration_per_top_action=1
)

import code 
code.interact(local=locals())

