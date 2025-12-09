# Copyright (c) 2022-2024, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to train RL agent with RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys

# local imports
# import cli_args  # isort: skip


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--num_search_agents",  type=int, default=16,       help="Number of search agents.") #64
parser.add_argument("--num_search_times",   type=int, default=800,      help="Number of search times.")
parser.add_argument("--training_iters",     type=int, default=100,      help="Training iterations.")
parser.add_argument("--round_per_dataset",  type=int, default=10,       help="Number of latest rounds per dataset.")
parser.add_argument("--puct",               type=bool, default=True,    help="PUCT exploration constant.")
parser.add_argument("--seed",               type=int, default=1,     help="Random seed.")

# ============================================= For Debugging =============================================
# parser.add_argument("--num_search_agents",  type=int, default=3,       help="Number of search agents.") #64
# parser.add_argument("--num_search_times",   type=int, default=1000,      help="Number of search times.")
# parser.add_argument("--training_iters",     type=int, default=1,      help="Training iterations.")
# parser.add_argument("--round_per_dataset",  type=int, default=10,       help="Number of latest rounds per dataset.")
# parser.add_argument("--puct",               type=bool, default=True,    help="PUCT exploration constant.")
# parser.add_argument("--seed",               type=int, default=1,     help="Random seed.")
# =========================================================================================================

args_cli, hydra_args = parser.parse_known_args()

"""Rest everything follows."""

import os
import torch
import queue
from datetime import datetime

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False

# =====================================================================================================
from torch.utils.data import Dataset, DataLoader, ConcatDataset
from torch.utils.tensorboard import SummaryWriter
import numpy as np
import rclpy
import time
import yaml
import json
import pandas as pd

# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
simulation_dir = os.path.abspath(os.path.join(project_root, "scripts", "simulation"))
bt_dir = os.path.abspath(os.path.join(project_root, "scripts", "bt"))

from gymEnv import Simple_MultiBTEnv
from mcts import MCTS
from network import RvNN_mem, RvNN

import psutil

process = psutil.Process(os.getpid())

if args_cli.seed is not None:
    SEED = args_cli.seed
    torch.manual_seed(SEED)
    np.random.seed(SEED)

class PeekableQueue(queue.Queue):
    def peek_all(self):
        """Return a list of all items in the queue without removing them."""
        with self.mutex:  # ensure thread-safety
            return list(self.queue)

class BTDataset(Dataset):
    def __init__(self, bt_strings, action_probs, rewards, device):
        """
        :param bt_strings: List of behavior tree strings
        :param action_probs: List of action probabilities (numpy or list)
        :param rewards: List of rewards (numpy or list)
        """
        self.bt_strings = bt_strings
        self.action_probs = action_probs
        self.rewards = rewards
        self.device = device

    def __len__(self):
        return len(self.bt_strings)

    def __getitem__(self, idx):
        # You can convert numpy arrays to torch.Tensor here if needed
        return (
            self.bt_strings[idx],
            torch.tensor(self.action_probs[idx], dtype=torch.float32, device=self.device),
            torch.tensor(self.rewards[idx], dtype=torch.float32, device=self.device)
        )
    
def log_mem(prefix=""):
    mem = process.memory_info().rss / 1024**2
    print(f"{prefix}Memory: {mem:.2f} MB")
    
def save_config_to_yaml(
    args,
    node_dict,
    nodes_limit,
    num_epochs,
    exploration_weight,
    model,
    optimizer,
    num_node_to_explore,
    epsilon,
    allow_duplicate_nodes,
    fitness_mode,
    l2_weight,
    log_dir,
    filename="config.yaml"
):
    # Extract optimizer info
    optimizer_config = {
        "class": optimizer.__class__.__name__,
        "lr": optimizer.param_groups[0].get("lr", None),
        "weight_decay": optimizer.param_groups[0].get("weight_decay", None),
        "betas": list(optimizer.param_groups[0].get("betas", None)),
        "eps": optimizer.param_groups[0].get("eps", None),
    }

    config = {
        "exploration_weight": exploration_weight,
        "num_search_agents": args.num_search_agents,
        "num_search_times": args.num_search_times,
        "training_iters": args.training_iters,
        "round_per_dataset": args.round_per_dataset,
        "seed": args.seed,
        "node_dict": node_dict,
        "nodes_limit": nodes_limit,
        "num_epochs": num_epochs,
        "num_node_to_explore": num_node_to_explore,
        "epsilon": epsilon,
        "allow_duplicate_nodes": allow_duplicate_nodes,
        "puct": args.puct,
        "fitness_mode": fitness_mode,
        "l2_weight": l2_weight,
        "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "model": {
            "class": model.__class__.__name__,
            "node_type_vocab_size": model.node_type_vocab_size,
            "embed_size": model.embed_size,
            "hidden_size": model.hidden_size,
            "action_size": model.action_size,
            "reward_head": model.reward_head,
            "device": str(model.device),
        },
        "optimizer": optimizer_config,
    }

    yaml_path = os.path.join(log_dir, filename)
    with open(yaml_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False)

    print(f"[INFO] Parameters + model + optimizer + training settings saved to {yaml_path}")
    return yaml_path

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

def dataset_generation(node_dict, nodes_limit, num_search_agents, num_search, policy_net, exploration_weight, num_node_to_explore = 10, epsilon = 0.7, allow_duplicate_nodes=True, fitness_mode="random", device='cuda:0', verbose = False, log_file=None):
    policy_net = policy_net.to(device)

    env = Simple_MultiBTEnv(node_dict, 
                            nodes_limit, 
                            num_envs=num_search_agents,
                            verbose=False)
    mcts = MCTS(env, policy_net, num_simulations=num_search, exploration_weight=exploration_weight, allow_duplicate_nodes=allow_duplicate_nodes, fitness_mode=fitness_mode, model_based=False, device=device)

    bt_string = ''

    bt_strings = []
    action_probs = []

    number_of_nodes = 0

    while True:
        # Calculate the temperature
        if number_of_nodes < num_node_to_explore:
            temperature = 1.0
        else:
            temperature = 1/(number_of_nodes - (num_node_to_explore - 1))

        # [1] Get the action probabilities from MCTS search ============================================
        action_prob = mcts.run_search(root_state=bt_string, PUCT=args_cli.puct, temperature=temperature, verbose=True) # Set verbose = True if want to see each search step run time
        
        # Store the sample data
        bt_strings.append(bt_string)
        action_probs.append(action_prob)

        # log the BT string to file if log_file is provided
        if log_file is not None:
            log_file.write(f"Step {number_of_nodes}: {bt_string}\n")
            log_file.flush()

        # [2] Select action based on epsilon-greedy strategy ==========================================
        if np.random.rand() < epsilon:
             # Sample an action according to the action probabilities
            selected_action = np.random.choice(len(action_prob), p=action_prob)
        else:
            max_action_prob = np.max(action_prob)
            best_action_indices = np.where(action_prob == max_action_prob)[0]

            # Randomly select one of the best indices
            selected_action = np.random.choice(best_action_indices)

        # Decode the selected action into node type and location
        if selected_action > 3:
            selected_nt = (selected_action - 4) % (len(node_dict.items()) - 1) + 1
            selected_loc = (selected_action - 4) // (len(node_dict.items()) - 1) + 1
        else:
            selected_nt = selected_action
            selected_loc = 0

        # [3] Modify the BT string based on the selected action ======================================
        bt_string = modify_bt(node_dict, bt_string, selected_nt, selected_loc)
        number_of_nodes += 1

        if verbose: log_mem(f"[INFO] Dataset {number_of_nodes}/{nodes_limit} << {bt_strings[-1]}, ({selected_nt}, {selected_loc})   ")

        if selected_nt == 0 or number_of_nodes >= nodes_limit:
            break

    for env_id in range(env.num_envs):
        mcts.env.set_bt(env_id=env_id, bt_string=bt_string)

    # Get the reward by runnung the BT in IsaacSim Simulation
    if verbose: print(f"[INFO] \tEvaluating {bt_string}")
    _, rews, _, infos =  env.evaluate_bt_in_sim()
    rew = np.mean(rews)
    if verbose: print(f"[INFO] \tFinished Evaluation >> reward: {rew}")

    # Convert rewards to numpy array
    rewards = np.array([rew] * len(bt_strings))
    
    # Convert list of np.array to a single np.array
    action_probs = np.array(action_probs)

    return BTDataset(bt_strings, action_probs, rewards, device=device)

if __name__ == "__main__":
    # setting device on GPU if available, else CPU
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # Specify Hyperparameters =======================================================================================
    # Node dictionary
    #             # Flow Control
    # node_dict = {   0 : '(0)', #sequence_node
    #                 1 : '(1)', #fallback_node
    #                 2 : '(2)', #parallel_node
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
    
    # node_dict = {   0 : '(0)', #sequence_node
    #                 1 : '(1)', #fallback_node
    #                 2 : '(2)', #parallel_node
    #                 # Behaviors
    #                 3 : 'a', #patrol_node
    #                 4 : 'b', #find_target_node
    #                 5 : 'c', #go_to_nearest_target
    #                 6 : 'e', #go_to_spawn_node
    #                 7 : 'f', #picking_object_node
    #                 8 : 'g', #drop_object_node
    #                 # Conditions
    #                 9 : 'B', #is_robot_at_the_spawn_node
    #                 10 : 'D', #are_object_existed_on_internal_map
    #                 11 : 'E', #are_object_nearby_node
    #                 12 : 'F', #is_object_in_hand_node
    #                 13 : 'H', #are_five_objects_at_spawn
    #                 # Specials
    #                 14 : None, #stop node
    #                 }
    
                    # Specials
    node_dict = {   0 : None,
                    1 : '(0)', #sequence_node
                    2 : '(1)', #fallback_node
                    3 : '(2)', #parallel_node
                    # Behaviors
                    4 : 'a', #patrol_node
                    5 : 'b', #find_target_node
                    6 : 'c', #go_to_nearest_target
                    # Conditions
                    7 : 'D', #are_object_existed_on_internal_map
                    }
    
    # Maximum of nodes in the BT
    nodes_limit = 10

    # Number of training epochs
    num_epochs = 10

    # Number of nodes to explore before switching to exploitation
    num_node_to_explore = 10

    # L2 regularization weight
    l2_weight = 1e-4

    # MMCGS Settings
    exploration_weight = 1.0    
    fitness_mode = "less_nodes" 
    allow_duplicate_nodes = True

    epsilon = 0.0

    # ===============================================================================================================

    # Instantiate the model
    model = RvNN(
        node_type_vocab_size=20,
        embed_size=4,  # was 64
        hidden_size=8, # was 128
        action_size=4 + (len(node_dict.items()) - 1) * (2 * nodes_limit - 1),    # Number of node types to choose from * Max insertion locations (50 * 2) - 1 
        device=device,
        reward_head=False,                      # Set to True if you want to include a reward head
    )

    # Optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)

    # TensorBoard writer setup
    log_dir = os.path.join(project_root, "logs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    writer = SummaryWriter(log_dir=log_dir)

    # BT log path
    bt_log_path = os.path.join(log_dir, f"bt_log.txt")
    log_file = open(bt_log_path, "w")
    print(f"[INFO] Logging BT strings to {bt_log_path}")

    global_step = 0
    start_time = time.time()
    dataset_queue = PeekableQueue()

    config_path = save_config_to_yaml(
        args=args_cli,
        node_dict=node_dict,
        nodes_limit=nodes_limit,
        num_epochs=num_epochs,
        exploration_weight=exploration_weight,
        model=model,
        optimizer=optimizer,
        num_node_to_explore=num_node_to_explore,
        allow_duplicate_nodes=allow_duplicate_nodes,
        epsilon=epsilon,
        fitness_mode=fitness_mode,
        l2_weight=l2_weight,
        log_dir=log_dir,
    )

    # path to store persistent dataset file (same directory as config.yaml)
    dataset_dir = os.path.dirname(config_path)
    csv_dataset_path = os.path.join(dataset_dir, "dataset.csv")
    json_dataset_path = os.path.join(dataset_dir, "dataset_all.json")

    # If you want a fresh dataset file each run, uncomment the following:
    # if os.path.exists(csv_dataset_path):
    #     os.remove(csv_dataset_path)
    # if os.path.exists(json_dataset_path):
    #     os.remove(json_dataset_path)

    for iter_i in range(args_cli.training_iters):
        print(f"[INFO] Iteration {iter_i + 1}/{args_cli.training_iters}")

        log_file.write(f"Iteration {iter_i + 1}/{args_cli.training_iters}\n")
        log_file.flush()

        # Generate dataset
        current_dataset = dataset_generation(
            node_dict,
            nodes_limit,
            num_search_agents=args_cli.num_search_agents,
            num_search=args_cli.num_search_times,
            policy_net=model,
            exploration_weight=exploration_weight,
            num_node_to_explore=num_node_to_explore,
            allow_duplicate_nodes=allow_duplicate_nodes,
            device=device,
            verbose=True,
            epsilon=epsilon,
            fitness_mode=fitness_mode,
            log_file=log_file
        )
        dataset_queue.put(current_dataset)

        # ------------------ Persist the newly generated dataset to disk (CSV + JSON snapshot) ------------------
        try:
            # gather arrays/lists from current_dataset
            bt_strings = list(current_dataset.bt_strings)
            # action_probs might be numpy array with shape (N, action_dim)
            action_probs_arr = np.array(current_dataset.action_probs)
            # convert to list-of-lists for JSON-friendly storing
            action_probs_list = [ap.tolist() for ap in action_probs_arr]
            rewards_arr = np.array(current_dataset.rewards).tolist()

            # create pandas DataFrame
            df = pd.DataFrame({
                "bt_string": bt_strings,
                "action_probs": [json.dumps(ap) for ap in action_probs_list],  # store as JSON string in CSV
                "reward": rewards_arr,
                "iteration": [int(iter_i + 1)] * len(bt_strings)  # 1-based iteration index saved
            })

            # Append to CSV (create if not exists)
            if not os.path.exists(csv_dataset_path):
                df.to_csv(csv_dataset_path, index=False)
                print(f"[INFO] Created dataset CSV at {csv_dataset_path} with {len(df)} rows (iter {iter_i+1}).")
            else:
                df.to_csv(csv_dataset_path, mode="a", header=False, index=False)
                print(f"[INFO] Appended {len(df)} rows to dataset CSV at {csv_dataset_path} (iter {iter_i+1}).")

            # Also save/overwrite a JSON snapshot of the accumulated datasets for quick programmatic load
            accumulated = []
            if os.path.exists(json_dataset_path):
                try:
                    with open(json_dataset_path, "r") as jf:
                        accumulated = json.load(jf)
                        if not isinstance(accumulated, list):
                            accumulated = []
                except Exception as e:
                    print(f"[WARN] Failed to load existing JSON snapshot (will recreate): {e}")
                    accumulated = []

            # extend with current iteration entries
            for bs, ap, rw in zip(bt_strings, action_probs_list, rewards_arr):
                accumulated.append({"bt_string": bs, "action_probs": ap, "reward": float(rw), "iteration": int(iter_i + 1)})

            # save JSON snapshot (atomic write)
            try:
                tmp_path = json_dataset_path + ".tmp"
                with open(tmp_path, "w") as jf:
                    json.dump(accumulated, jf, ensure_ascii=False)
                os.replace(tmp_path, json_dataset_path)
                print(f"[INFO] Saved JSON dataset snapshot at {json_dataset_path} (total samples: {len(accumulated)}).")
            except Exception as e:
                print(f"[WARN] Failed to save JSON dataset snapshot: {e}")

        except Exception as e:
            print(f"[WARN] Failed to persist dataset for iteration {iter_i + 1}: {e}")
        # --------------------------------------------------------------------------------------------------------

        if dataset_queue.qsize() > args_cli.round_per_dataset:
            dataset_queue.get()

        dataset = ConcatDataset(dataset_queue.peek_all())

        train_loader = DataLoader(dataset, batch_size=4, shuffle=True)

        # Train and log metrics
        avg_loss = model.train_loop(
            train_loader=train_loader,
            optimizer=optimizer,
            num_epochs=num_epochs,
            l2_weight=l2_weight,
            writer=writer,
            global_step=global_step
        )

        # Log final evaluation reward
        # current_dataset.rewards is a numpy array on device or on CPU; convert safely
        last_reward_val = current_dataset.rewards[-1]
        if isinstance(last_reward_val, torch.Tensor):
            last_reward_val = last_reward_val.item()
        writer.add_scalar("Eval/FinalReward", float(last_reward_val), iter_i)

        # Save model after each iteration (optional: adjust to save best only)
        model_path = os.path.join(log_dir, f"rvnn_iter{iter_i:03d}.pt")
        torch.save(model.state_dict(), model_path)
        print(f"[INFO] Model saved at {model_path}")

        global_step += num_epochs  # assuming 20 epochs per iteration

    # Save final model
    final_model_path = os.path.join(log_dir, "rvnn_final.pt")
    torch.save(model.state_dict(), final_model_path)
    print(f"[INFO] Final model saved at {final_model_path}")

    # Close TensorBoard writer
    writer.close()

    print(f"Final Results: {current_dataset.bt_strings[-1]}")
    print(f"Total elapsed time: {time.time() - start_time:.2f} seconds")

    log_file.close()

    # Use this command to view log -> tensorboard --logdir logs/
