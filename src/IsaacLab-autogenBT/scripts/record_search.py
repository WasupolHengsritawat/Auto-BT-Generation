import argparse
import sys
import os
import torch
import numpy as np

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, ".."))

logs_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))

date_time = "2025-05-12_21-50-39"
model_name = "rvnn_iter354"
full_model_name = f"{model_name}.pt"

model_path = os.path.join(logs_dir, date_time, model_name)

sys.path.insert(0, project_root)
sys.path.insert(0, script_dir)

from learning.network import RvNN, RvNN_mem
from learning.mcts import MCTS
from learning.gymEnv import MultiBTEnv

device = "cuda"

num_search_agents = 100
num_search = 200

num_node_to_explore = 10

# Flow Control
node_dict = {   0 : '(0)', #patrol_node
                1 : '(1)', #find_target_node
                2 : '(2)', #go_to_nearest_target
            # Behaviors
                3 : 'a', #patrol_node
                4 : 'b', #find_target_node
                5 : 'c', #go_to_nearest_target
                6 : 'e', #go_to_spawn_node
                7 : 'f', #picking_object_node
                8 : 'g', #drop_object_node
            # Conditions
                9 : 'B', #is_robot_at_the_spawn_node
                10 : 'D', #are_object_existed_on_internal_map
                11 : 'E', #are_object_nearby_node
                12 : 'F', #is_object_in_hand_node
                13 : 'G', #is_nearby_object_not_at_goal
                14 : 'H', #are_five_objects_at_spawn
            # Specials
                15 : None, #stop node
            }

# Maximum of nodes in the BT
nodes_limit = 25

# Number of training epochs
num_epochs = 20

# Simulation configurations
# number_of_target_to_success=3 
# number_of_spawned_objects=8
# sim_step_limit=160000

number_of_target_to_success=1 
number_of_spawned_objects=10
sim_step_limit=20000

def modify_bt(node_dict, current_bt, node_type, node_location):
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
        if node_location == 0 and node_type in [0, 1, 2]:
            return f'({node_type}' + bt_string + ')'
        
        else:
            # Iterate over all potential insertion positions (0 to len(s))
            valid_indices = [j for j in range(1,len(bt_string)) if j == len(bt_string) or not bt_string[j].isdigit()]

            if 1 <=  node_location < len(valid_indices) + 1:
                # Get the actual index in the string where we want to insert the char.
                insert_index = valid_indices[node_location - 1]

                # Return the new string with the character inserted.
                return bt_string[:insert_index] + node + bt_string[insert_index:]
        
    return current_bt

# Instantiate the model
model = RvNN(
    node_type_vocab_size=20,  # 0 to 19
    embed_size=64,
    hidden_size=128,
    action1_size=len(node_dict.items()),    # Number of node types to choose from
    action2_size=2*nodes_limit,             # Max insertion locations (50 * 2) - 1 + 1
    device=device
)

# Optimizer
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)

policy_net = model.to(device)

env = MultiBTEnv(node_dict, 
                    nodes_limit, 
                    num_envs=num_search_agents,
                    sim_step_limit=sim_step_limit, device=device, verbose=False)
mcts = MCTS(env, policy_net, num_simulations=num_search, exploration_weight=1.0, device=device)

bt_string = ''

bt_strings = []
action1_probs = []
action2_probs = []

number_of_nodes = 0
count = 0

while True:
    count += 1
    # Calculate the temperature
    if number_of_nodes < num_node_to_explore:
        temperature = 1.0
    else:
        temperature = 1/(number_of_nodes - (num_node_to_explore - 1))

    # Get the action probabilities from MCTS search
    save_path = os.path.join(logs_dir, date_time, model_name, f"mcts_tree_{count}.json")
    nt_probs, loc_probs = mcts.run_search(root_state=bt_string,temperature=temperature, verbose=False, export_path=save_path) # Set verbose = True if want to see each search step run time

    # print(loc_probs)

    # Store the sample data
    bt_strings.append(bt_string)
    action1_probs.append(nt_probs)
    action2_probs.append(loc_probs)
    
    max_nt_probs = np.max(nt_probs)
    max_loc_probs = np.max(loc_probs)

    best_nt_indices = np.where(nt_probs == max_nt_probs)[0]
    best_loc_indices = np.where(loc_probs == max_loc_probs)[0]

    # Randomly select one of the best indices
    selected_nt = np.random.choice(best_nt_indices)
    selected_loc = np.random.choice(best_loc_indices)

    bt_string = modify_bt(node_dict, bt_string, selected_nt, selected_loc)
    number_of_nodes += 1

    if selected_nt == (len(node_dict.items()) - 1) or number_of_nodes >= nodes_limit:
        break