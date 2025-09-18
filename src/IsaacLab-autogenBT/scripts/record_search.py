import argparse
import sys
import os
import torch
import yaml
import numpy as np

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, ".."))

logs_dir = os.path.abspath(os.path.join(script_dir, "..", "logs"))

date_time = "2025-09-04_18-02-46-fibo2-ind-064-128-seedNA-199iters"
model_name = "rvnn_iter198"
full_model_name = f"{model_name}.pt"

SEED = 1
torch.manual_seed(SEED)
np.random.seed(SEED)

model_path = os.path.join(logs_dir, date_time, full_model_name)
yaml_path = os.path.join(logs_dir, date_time, "config.yaml")

sys.path.insert(0, project_root)
sys.path.insert(0, script_dir)

from learning.network import RvNN, RvNN_mem
from learning.mcts import MCTS
from learning.gymEnv import Simple_MultiBTEnv

device = "cuda"

def load_config_from_yaml(config_path, device=None):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # --- CLI arguments ---
    args = {
        "num_search_agents": config["num_search_agents"],
        "num_search_times": config["num_search_times"],
        "training_iters": config["training_iters"],
        "round_per_dataset": config["round_per_dataset"],
        "seed": config["seed"],
    }

    # --- Node dictionary & BT parameters ---
    node_dict = config["node_dict"]
    nodes_limit = config["nodes_limit"]
    num_epochs = config["num_epochs"]
    num_node_to_explore = config["num_node_to_explore"]
    l2_weight = config["l2_weight"]
    exploration_weight = config["exploration_weight"]

    # --- Model reconstruction ---
    model_cfg = config["model"]
    model = RvNN(
        node_type_vocab_size=model_cfg["node_type_vocab_size"],
        embed_size=model_cfg["embed_size"],
        hidden_size=model_cfg["hidden_size"],
        action1_size=model_cfg["action1_size"],
        action2_size=model_cfg["action2_size"],
        device=device or torch.device(model_cfg["device"]),
        reward_head=model_cfg["reward_head"],
    )

    # --- Optimizer reconstruction ---
    opt_cfg = config["optimizer"]
    optimizer_class = getattr(torch.optim, opt_cfg["class"])  # e.g. Adam, SGD, etc.
    optimizer = optimizer_class(
        model.parameters(),
        lr=opt_cfg.get("lr", 1e-3),
        weight_decay=opt_cfg.get("weight_decay", 0),
        betas=opt_cfg.get("betas", (0.9, 0.999)),
        eps=opt_cfg.get("eps", 1e-8),
    )

    # --- Wrap everything into one dict ---
    return {
        "args": args,
        "node_dict": node_dict,
        "nodes_limit": nodes_limit,
        "num_epochs": num_epochs,
        "num_node_to_explore": num_node_to_explore,
        "l2_weight": l2_weight,
        "exploration_weight": exploration_weight,
        "timestamp": config["timestamp"],
        "model": model,
        "optimizer": optimizer,
        "raw_config": config,  # keep raw in case you want direct access
    } 

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

cfg = load_config_from_yaml(yaml_path, device=device)

SEED = cfg["args"]["seed"]
torch.manual_seed(SEED)
np.random.seed(SEED)

num_search_agents = cfg["args"]["num_search_agents"]
num_search = cfg["args"]["num_search_times"]
nodes_limit = cfg["nodes_limit"]
num_node_to_explore = cfg["num_node_to_explore"]
node_dict = cfg["node_dict"]
exploration_weight = cfg["exploration_weight"]

model = cfg["model"]
model.load_state_dict(torch.load(model_path))
model.eval()

policy_net = model.to(device)

env = Simple_MultiBTEnv(node_dict, 
                        nodes_limit, 
                        num_envs=num_search_agents,
                        verbose=False)
mcts = MCTS(env, policy_net, num_simulations=num_search, exploration_weight=exploration_weight, device=device)

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

    print(f"[INFO] Dataset {number_of_nodes}/{nodes_limit} << {bt_strings[-1]}, ({selected_nt}, {selected_loc})")

    if selected_nt == (len(node_dict.items()) - 1) or number_of_nodes >= nodes_limit:
        break

print(f"Final BT >> {bt_string}")