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
parser.add_argument("--num_search_agents", type=int, default=1, help="Number of search agents.") #64
parser.add_argument("--num_search_times", type=int, default=200, help="Number of search times.")
parser.add_argument("--training_iters", type=int, default=100, help="Training iterations.")
parser.add_argument("--env_spacing", type=int, default=40, help="Space between environments.")

args_cli, hydra_args = parser.parse_known_args()

"""Rest everything follows."""

import os
import torch
from datetime import datetime

torch.backends.cuda.matmul.allow_tf32 = True
torch.backends.cudnn.allow_tf32 = True
torch.backends.cudnn.deterministic = False
torch.backends.cudnn.benchmark = False

# =====================================================================================================
from torch.utils.data import Dataset, DataLoader, random_split
from torch.utils.tensorboard import SummaryWriter
import numpy as np
import rclpy
import time

# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
simulation_dir = os.path.abspath(os.path.join(project_root, "scripts", "simulation"))
bt_dir = os.path.abspath(os.path.join(project_root, "scripts", "bt"))

from gymEnv import Simple_MultiBTEnv
from mcts import MCTS
from network import RvNN_mem, RvNN

class BTDataset(Dataset):
    def __init__(self, bt_strings, action1_probs, action2_probs, rewards, device):
        """
        :param bt_strings: List of behavior tree strings
        :param action1_probs: List of node type distributions (numpy or list)
        :param action2_probs: List of location distributions (numpy or list)
        """
        self.bt_strings = bt_strings
        self.action1_probs = action1_probs
        self.action2_probs = action2_probs
        self.rewards = rewards
        self.device = device

    def __len__(self):
        return len(self.bt_strings)

    def __getitem__(self, idx):
        # You can convert numpy arrays to torch.Tensor here if needed
        return (
            self.bt_strings[idx],
            torch.tensor(self.action1_probs[idx], dtype=torch.float32, device=self.device),
            torch.tensor(self.action2_probs[idx], dtype=torch.float32, device=self.device),
            torch.tensor(self.rewards[idx], dtype=torch.float32, device=self.device)
        )
    
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

def dataset_generation(node_dict, nodes_limit, num_search_agents, num_search, policy_net, num_node_to_explore = 10, device='cuda:0', verbose = False):
    policy_net = policy_net.to(device)

    env = Simple_MultiBTEnv(node_dict, 
                            nodes_limit, 
                            num_envs=num_search_agents,
                            verbose=False)
    mcts = MCTS(env, policy_net, num_simulations=num_search, exploration_weight=1.0, model_based=False, device=device)

    bt_string = ''

    bt_strings = []
    action1_probs = []
    action2_probs = []

    number_of_nodes = 0

    while True:
        # Calculate the temperature
        if number_of_nodes < num_node_to_explore:
            temperature = 1.0
        else:
            temperature = 1/(number_of_nodes - (num_node_to_explore - 1))

        # Get the action probabilities from MCTS search
        nt_probs, loc_probs = mcts.run_search(root_state=bt_string,temperature=temperature, verbose=False) # Set verbose = True if want to see each search step run time

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

        if verbose: print(f"[INFO] Dataset {number_of_nodes}/{nodes_limit} << {bt_strings[-1]}, ({selected_nt}, {selected_loc})")

        if selected_nt == (len(node_dict.items()) - 1) or number_of_nodes >= nodes_limit:
            break

    for env_id in range(env.num_envs):
        mcts.set_bt(env_id=env_id, bt_string=bt_string)

    # Get the reward by runnung the BT in IsaacSim Simulation
    if verbose: print(f"[INFO] \tEvaluating {bt_string}")
    _, rews, _, infos =  env.evaluate_bt_in_sim()
    rew = np.mean(rews)
    if verbose: print(f"[INFO] \tFinished Evaluation >> reward: {rew}")

    # Convert rewards to numpy array
    rewards = np.array([rew] * len(bt_strings))
    
    # Convert list of np.array to a single np.array
    action1_probs = np.array(action1_probs)
    action2_probs = np.array(action2_probs)

    return BTDataset(bt_strings, action1_probs, action2_probs, rewards, device=device)

if __name__ == "__main__":
    # setting device on GPU if available, else CPU
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # Specify Hyperparameters =======================================================================================
    # Node dictionary
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
    
    node_dict = {   0 : '0', #patrol_node
                    1 : '1', #find_target_node
                    2 : '2', #go_to_nearest_target
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
                    13 : 'H', #are_five_objects_at_spawn
                    # Specials
                    14 : None, #stop node
                    }
    
    # Maximum of nodes in the BT
    nodes_limit = 25

    # Number of training epochs
    num_epochs = 20

    # ===============================================================================================================

    # Instantiate the model
    model = RvNN(
        node_type_vocab_size=20,
        embed_size=64,
        hidden_size=128,
        action1_size=len(node_dict.items()),    # Number of node types to choose from
        action2_size=2*nodes_limit,             # Max insertion locations (50 * 2) - 1 + 1
        device=device,
        reward_head=False,                  # Set to True if you want to include a reward head
    )

    # Optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3, weight_decay=1e-4)

    # TensorBoard writer setup
    log_dir = os.path.join(project_root, "logs", datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
    writer = SummaryWriter(log_dir=log_dir)

    global_step = 0
    start_time = time.time()
    for iter_i in range(args_cli.training_iters):
        print(f"[INFO] Iteration {iter_i + 1}/{args_cli.training_iters}")

        # Generate dataset
        dataset = dataset_generation(
            node_dict,
            nodes_limit,
            num_search_agents=args_cli.num_search_agents,
            num_search=args_cli.num_search_times,
            policy_net=model,
            device=device,
            verbose=True
        )

        train_loader = DataLoader(dataset, batch_size=4, shuffle=True)

        # Train and log metrics
        avg_loss = model.train_loop(
            train_loader=train_loader,
            optimizer=optimizer,
            num_epochs=num_epochs,
            l2_weight=1e-4,
            writer=writer,
            global_step=global_step
        )

        # Log final evaluation reward
        writer.add_scalar("Eval/FinalReward", dataset.rewards[0].item(), iter_i)

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

    print(f"Final Results: {dataset.bt_strings[-1]}")
    print(f"Total elapsed time: {time.time() - start_time:.2f} seconds")
    rclpy.try_shutdown()

    # Use this command to view log -> tensorboard --logdir logs/
