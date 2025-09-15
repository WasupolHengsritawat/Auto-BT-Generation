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

date_time = "2025-09-11_15-05-17-fibo5-dep-seed01-200iters"
model_name = "rvnn_iter199"
full_model_name = f"{model_name}.pt"

SEED = 1
torch.manual_seed(SEED)
np.random.seed(SEED)

model_path = os.path.join(logs_dir, date_time, full_model_name)

sys.path.insert(0, project_root)
sys.path.insert(0, script_dir)

from learning.network import RvNN, RvNN_mem
from learning.mcts import MCTS
from learning.gymEnv import Simple_MultiBTEnv

device = "cuda"

num_search_agents = 16
num_search = 800

num_node_to_explore = 10

# # Node dictionary
#             # Flow Control
# node_dict = {   0 : '(0)', #patrol_node
#                 1 : '(1)', #find_target_node
#                 2 : '(2)', #go_to_nearest_target
#             # Behaviors
#                 3 : 'a', #patrol_node
#                 4 : 'b', #find_target_node
#                 5 : 'c', #go_to_nearest_target
#                 6 : 'd', #go_to_charger_node
#                 7 : 'e', #go_to_spawn_node
#                 8 : 'f', #picking_object_node
#                 9 : 'g', #drop_object_node
#                 10: 'h', #charge_node
#             # Conditions
#                 11 : 'A', #is_robot_at_the_charger_node
#                 12 : 'B', #is_robot_at_the_spawn_node
#                 13 : 'C', #is_battery_on_proper_level
#                 14 : 'D', #are_object_existed_on_internal_map
#                 15 : 'E', #are_object_nearby_node
#                 16 : 'F', #is_object_in_hand_node
#                 17 : 'G', #is_nearby_object_not_at_goal
#                 18 : 'H', #are_five_objects_at_spawn
#             # Specials
#                 19 : None, #stop node
#             }

                # Specials
node_dict = {   0 : None,
                1 : '(0)', #patrol_node
                2 : '(1)', #find_target_node
                3 : '(2)', #go_to_nearest_target
                # Behaviors
                4 : 'a', #patrol_node
                5 : 'b', #find_target_node
                6 : 'c', #go_to_nearest_target
                7 : 'e', #go_to_spawn_node
                8 : 'f', #picking_object_node
                9 : 'g', #drop_object_node
                # Conditions
                10 : 'B', #is_robot_at_the_spawn_node
                11 : 'D', #are_object_existed_on_internal_map
                12 : 'E', #are_object_nearby_node
                13 : 'F', #is_object_in_hand_node
                14 : 'H', #are_five_objects_at_spawn
                }

# Maximum of nodes in the BT
nodes_limit = 25

# Number of training epochs
num_epochs = 20

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

# Instantiate the model
model = RvNN(
    node_type_vocab_size=20,
    embed_size=32,  # was 64
    hidden_size=64, # was 128
    action_size=4 + (len(node_dict.items()) - 1) * (2 * nodes_limit - 1),    # Number of node types to choose from * Max insertion locations (50 * 2) - 1 
    device=device,
    reward_head=False,                      # Set to True if you want to include a reward head
)
model.load_state_dict(torch.load(model_path))
model.eval()

# Optimizer
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)

policy_net = model.to(device)

env = Simple_MultiBTEnv(node_dict, 
                        nodes_limit, 
                        num_envs=num_search_agents,
                        verbose=False)
mcts = MCTS(env, policy_net, num_simulations=num_search, exploration_weight=1.0, device=device)

bt_string = ''

bt_strings = []
action_probs = []


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
    action_prob = mcts.run_search(root_state=bt_string,temperature=temperature, verbose=False, export_path=save_path) # Set verbose = True if want to see each search step run time
    
    # Store the sample data
    bt_strings.append(bt_string)
    action_probs.append(action_prob)

    max_action_prob = np.max(action_prob)

    best_action_indices = np.where(action_prob == max_action_prob)[0]

    # Randomly select one of the best indices
    selected_action = np.random.choice(best_action_indices)

    if selected_action > 3:
        selected_nt = (selected_action - 3) % (len(node_dict.items()) - 1)
        selected_loc = (selected_action - 3) // (len(node_dict.items()) - 1) + 1
    else:
        selected_nt = selected_action
        selected_loc = 0

    bt_string = modify_bt(node_dict, bt_string, selected_nt, selected_loc)
    number_of_nodes += 1

    print(f"[INFO] Dataset {number_of_nodes}/{nodes_limit} << {bt_strings[-1]}, ({selected_nt}, {selected_loc})")

    if selected_nt == 0 or number_of_nodes >= nodes_limit:
        break

print(f"Final BT >> {bt_string}")