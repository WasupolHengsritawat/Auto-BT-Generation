###
#  Monte-Carlo Tree Search
###
import json
import numpy as np
from tqdm import trange
import time
import os

class MCTSNode:
    def __init__(self, state, env, policy_net, id = 0, parent_edge=None):
        """
        Represents a single state (node) in the MCTS tree for Behavior Tree construction.

        :param state: String representation of the current Behavior Tree (BT).
        :param env: The MultiBTEnv environment instance.
        :param policy_net: Neural network (RvNN) that outputs action probabilities.
        :param parent_edge: The edge from the parent node leading to this node.
        """
        self.id = id
        self.state = state                  # BT string for this node
        self.env = env                      # OpenGym Environment
        self.policy_net = policy_net        # RvNN model for policy and value
        self.is_terminated = False          # Flag to indicate if this node is terminal
        self.parent_edge = parent_edge      # Parent node
        self.reward = None

        # Get all possible actions
        bt_string = state
        valid_locs_on_string = [j for j in range(1,len(bt_string)) if j == len(bt_string) or not bt_string[j].isdigit()] # Find valid location on BT string
        valid_locs = range(len(valid_locs_on_string))

        # Define all possible actions as (node_type, node_location) tuples
        # The node_location = 0 means the location of the parent node while the other n locations are the locations of the nth child node
        if bt_string == '':
            # If the BT is empty, only a location of the root node is valid
            self.all_actions = [(nt, 1) for nt in range(1, self.env.num_node_types)]
        else:
            self.all_actions = [(nt, 0) for nt in range(4)] + [(nt, loc) for loc in range(1, len(valid_locs)) for nt in range(1, self.env.num_node_types)]

        # If there are no valid locations, only the stop action is valid
        if (0, 0) not in self.all_actions:
            self.all_actions.append((0, 0))

        # Get prior probabilities from the policy network
        action_probs, pred_rew = self.policy_net.predict(state)

        # Ensure probs are detached from GPU
        action_probs = action_probs.detach().cpu().numpy()

        # Initialize edges for each action
        self.edges = []
        for nt, loc in self.all_actions:
            if loc == 0:
                prior=float(action_probs[nt])
            else:
                prior=float(action_probs[3 + nt + (self.env.num_node_types - 1) * (loc - 1)])

            self.edges.append(MCTSEdge(
                    parent=self, 
                    action=(nt, loc), 
                    prior=prior))


class MCTSEdge:
    def __init__(self, parent, action, prior):
        """
        Represents an edge in the MCTS tree, connecting a parent node to a child.

        :param parent: MCTSNode that this edge originates from.
        :param action: The (node_type, node_location) tuple this edge represents.
        :param prior: Prior probability for this action (from the policy network).
        """
        self.parent = parent
        self.child = None

        self.action = action
        self.prior = prior
        self.cum_reward = 0.0
        self.visits = 0
        self.q = 0.0

class MCTS:
    def __init__(self, env, policy_net, num_simulations=50, exploration_weight=1.0, model_based = False, device='cpu'):
        """
        Model-based Monte Carlo Tree Search with PUCT for Behavior Tree generation.

        :param env: MultiBTEnv environment instance.
        :param policy_net: Neural network to predict action probabilities.
        :param num_simulations: Number of simulations per search.
        :param exploration_weight: Weight for exploration in the PUCT formula.
        :param device: Compute device ('cpu' or 'cuda').
        """
        self.env = env
        self.policy_net = policy_net.to(device)
        self.num_simulations = num_simulations
        self.exploration_weight = exploration_weight
        self.device = device
        self.node_id = 0
        self.model_based = model_based

    def run_search(self, root_state, temperature=1.0, dirichlet_noise_at_root=True, verbose=False, export_path=None):
        """
        Perform Monte Carlo Tree Search (MCTS) from a shared root Behavior Tree (BT) state.

        This function runs multiple simulations, where each simulation:
        1. Selects promising leaf nodes using the PUCT formula.
        2. Expands each selected leaf by simulating an action.
        3. Evaluates the new BTs via simulated rollouts to obtain rewards.
        4. Backpropagates the rewards to update edge statistics.

        After simulations, it aggregates the visit counts at the root to produce
        probability distributions over node types and insertion locations, optionally
        smoothed using a temperature-scaled softmax.

        :param root_state: Initial Behavior Tree string used as the root of the search.
        :param temperature: Softmax temperature to control exploration vs. exploitation
                            in the final probability distributions. Lower values favor
                            higher-probability actions.

        :return:
            - nt_prob (np.ndarray): Normalized visit-based probability distribution over node types.
            - loc_prob (np.ndarray): Normalized visit-based probability distribution over node locations.
        """
        root = MCTSNode(state=root_state, env=self.env, policy_net=self.policy_net)
        # >> print(f"Root possible actions: {root.all_actions}")

        if verbose:
            select_time_elapsed_avg = 0
            expand_time_elapsed_avg = 0
            evaluate_time_elapsed_avg = 0
            backpropagate_time_elapsed_avg = 0

        progress = trange(self.num_simulations, desc="[MCTS] Running search")
        for i in progress:
            progress.set_postfix(iteration=i)

            # Select a leaf node
            # print('Selecting...')
            if verbose: start_time = time.time()
            selected_edges = self.select(root, dirichlet_noise_at_root=dirichlet_noise_at_root)
            if verbose: select_time = time.time()

            # Expand the selected leaf node
            # print('Expanding...')
            self.expand(selected_edges)
            leave_nodes = [edge.child for edge in selected_edges]
            if verbose: expand_time = time.time()

            # Evaluate the expanded nodes
            # print('Evaluating...')
            rewards = self.evaluate(leave_nodes)
            if verbose: evaluate_time = time.time()

            # Backpropagate the results
            # print('Backpropagating...')
            self.backpropagate(leave_nodes, rewards)
            if verbose: backpropagate_time = time.time()

            if verbose:
                select_time_elapsed_avg += select_time - start_time
                expand_time_elapsed_avg += expand_time - select_time
                evaluate_time_elapsed_avg += evaluate_time - expand_time
                backpropagate_time_elapsed_avg += backpropagate_time - evaluate_time

        if verbose:
            select_time_elapsed_avg /= self.num_simulations
            expand_time_elapsed_avg /= self.num_simulations
            evaluate_time_elapsed_avg /= self.num_simulations
            backpropagate_time_elapsed_avg /= self.num_simulations
            print(f"Average time elapsed for selection: {select_time_elapsed_avg:.4f}s")
            print(f"Average time elapsed for expansion: {expand_time_elapsed_avg:.4f}s")
            print(f"Average time elapsed for evaluation: {evaluate_time_elapsed_avg:.4f}s")
            print(f"Average time elapsed for backpropagation: {backpropagate_time_elapsed_avg:.4f}s")

        # Collect visit counts and probabilities
        action_probs = np.zeros(4 + (self.env.num_node_types - 1) * (self.env.max_location_size - 1))

        for edge in root.edges:
            nt, loc = edge.action

            if loc == 0:
                index = nt
            else:
                index = 3 + nt + (self.env.num_node_types - 1) * (loc - 1)

            action_probs[index] += edge.visits

        action_prob_numerator = [action_prob ** (1/(temperature)) for action_prob in action_probs] 
        action_prob_denominator = sum(action_prob_numerator)
        action_probs = [num / action_prob_denominator for num in action_prob_numerator]

        # >> print(f"action_probs: {action_probs}")

        if export_path:
            self.export_tree(root, export_path)

        return action_probs

    def select(self, node, dirichlet_noise_at_root=True):    
        """
        Selects edges using PUCT from the root to a leaf for each agent.

        :param node: Root node.
        :param dirichlet_noise_at_root: If True, applies noise to root node priors.
        :return: List of selected edges (one per agent).
        """
        epsilon = 0.25
        selected_edges = []

        for _ in range(self.env.num_envs):
            is_root_flag = dirichlet_noise_at_root
            while True:
                # Add Dirichlet noise at root node
                if is_root_flag:
                    actual_prior = []
                    dirichlet_noise = np.random.dirichlet([0.03] * len(node.edges))
                    for i, edge in enumerate(node.edges):
                        actual_prior.append(edge.prior)  # store original prior
                        edge.prior = float((1 - epsilon) * edge.prior + epsilon * dirichlet_noise[i])

                # Select the edge with the highest PUCT value
                node_visits = sum([edge.visits for edge in node.edges])
                scores = np.array([
                            float(edge.q + self.exploration_weight * edge.prior * np.sqrt(node_visits) / (1 + edge.visits))
                            for edge in node.edges
                        ])
                max_score = np.max(scores)

                # Get indices of all edges with the maximal score
                best_indices = np.where(scores == max_score)[0]

                # Randomly select one of the best indices
                selected_index = np.random.choice(best_indices)

                selected_edge = node.edges[selected_index]
                
                # Set prior probabilities back to original
                if is_root_flag:
                    for i, edge in enumerate(node.edges):
                        edge.prior = actual_prior[i]
                    is_root_flag = False
                
                # Check if already reached a leaf node
                if selected_edge.child is None or selected_edge.child.is_terminated:
                    break
                
                # Traverse to the child node
                node = selected_edge.child
            
            selected_edges.append(selected_edge)

        return selected_edges

    def expand(self, edges):
        """
        Expands leaf nodes by performing their selected actions and creating children.

        :param edges: List of selected edges to expand.
        """
        actions = [edge.action for edge in edges]

        for env_id in range(self.env.num_envs):
            self.env.set_bt(env_id=env_id, bt_string=edges[env_id].parent.state)
        
        obs, _, dones, infos =  self.env.step_without_sim(actions)
        
        for env_id in range(self.env.num_envs):  
            # Check if the child node is not already created
            if edges[env_id].child is None:
                # Create a new child node for each selected edge
                self.node_id += 1
                edges[env_id].child = MCTSNode(state=obs[env_id], env=self.env, policy_net=self.policy_net, parent_edge=edges[env_id], id=self.node_id)
                
                if dones[env_id]:
                    edges[env_id].child.is_terminated = True

    def evaluate(self, nodes):
        """
        Evaluates the expanded nodes using the policy network.

        :param nodes: Nodes to evaluate.
        :return: List of rewards for each node.
        """
        if self.model_based:
            return self._evaluate_modelbased(nodes)
        
        return self._evaluate_modelfree(nodes)

    def _evaluate_modelfree(self, nodes):
        """
        Evaluates completed BTs using the simulation.

        :param nodes: Nodes with fully expanded BTs.
        :return: List of simulation-derived rewards.
        """
        states = [node.state for node in nodes]
        dones = [False for _ in range(self.env.num_envs)]

        # Make sure that the BT string is matched with the state in leaf node
        for env_id in range(self.env.num_envs):
            self.env.set_bt(env_id=env_id, bt_string=states[env_id])

        # print(f"[INFO] \tBTs after expand: {states}")
            
        # Expand the BT until it reaches a terminal state (stop action or max depth)
        while True:
            actions = []
            for env_id in range(self.env.num_envs):
                state = states[env_id]
                
                # If the BT Contruction is done, select stop action
                if nodes[env_id].is_terminated or dones[env_id]:
                    actions.append((0, 0))
                    continue

                # Get the action probablities from the policy network
                action_probs, _ = self.policy_net.predict(state) 

                # Convert torch tensors to numpy arrays
                action_probs = action_probs.detach().cpu().numpy()
  
                # Get all possible actions
                bt_string = state
                valid_locs_on_string = [j for j in range(1,len(bt_string)) if j == len(bt_string) or not bt_string[j].isdigit()] # Find valid location on BT string
                valid_locs = range(len(valid_locs_on_string))

                # Define all possible actions as (node_type, node_location) tuples
                # The node_location = 0 means the location of the parent node while the other n locations are the locations of the nth child node
                all_actions = [(0, 0)]
                if bt_string == '':
                    # If the BT is empty, only a location of the root node is valid
                    all_actions += [(nt, 1) for nt in range(1, self.env.num_node_types)]
                else:
                    all_actions += [(nt, 0) for nt in range(1, 4)] + [(nt, loc) for loc in range(1, len(valid_locs) + 1) for nt in range(1, self.env.num_node_types)]

                # Select the best action according to its joint probability
                probs = []
                for nt, loc in all_actions:
                    if loc == 0:
                        probs.append(float(action_probs[nt]))
                    else:
                        probs.append(float(action_probs[3 + nt + (self.env.num_node_types - 1) * (loc - 1)]))

                best_ind = np.where(probs == np.max(probs))[0]
                selected_ind = np.random.choice(best_ind)

                actions.append(all_actions[selected_ind])

            # Perform the action in the environment (add node to BT)
            obs, _, dones, infos =  self.env.step_without_sim(actions) 

            # Update the state for each node
            states = obs

            # Check if the BTs construction is done
            if all(dones):
                break

        # print(f"[INFO] \tBTs constructed: {states}")

        # Get the reward by runnung the BT in IsaacSim Simulation
        _, rews, _, infos =  self.env.evaluate_bt_in_sim()

        return rews

    def _evaluate_modelbased(self, nodes):
        """
        Evaluates completed BTs using the predicted rewards from the network.

        :param nodes: Nodes with fully expanded BTs.
        :return: List of simulation-derived rewards.
        """
        states = [node.state for node in nodes]
            
        # Evaluate the BTs using the policy network
        rews = []
        for env_id in range(self.env.num_envs):
            state = states[env_id]
            nt_probs, loc_probs, pred_rew = self.policy_net.predict(state)
            rew = float(pred_rew.detach().cpu().item())

            nodes[env_id].reward = rew
            rews.append(rew)

        return rews

    def backpropagate(self, nodes, rewards):
        """
        Backpropagates the reward from leaf to root through visited edges.

        :param nodes: Final leaf nodes.
        :param rewards: Rewards associated with each leaf.
        """
        for env_id in range(self.env.num_envs):
            node = nodes[env_id]        # leaf node
            reward = rewards[env_id]    # reward for the leaf node
   
            while True:
                # Find the traversed edge
                traversed_edge = node.parent_edge

                # If the traversed edge is None, we reached the root
                if traversed_edge is None:
                    break

                # Update the edge statistics
                traversed_edge.visits += 1
                traversed_edge.cum_reward += reward
                traversed_edge.q = traversed_edge.cum_reward / traversed_edge.visits

                # Update the node to its parent
                node = traversed_edge.parent

    def export_tree(self, root, save_path="mcts_tree.json"):
        """
        Export the MCTS tree starting from root to a JSON file.
        """
        visited = set()
        nodes = []
        edges = []

        def dfs(node):
            if node.id in visited:
                return
            visited.add(node.id)

            nodes.append({
                "id": node.id,
                "state": node.state,
                "reward": node.reward,
                "is_terminal": node.is_terminated
            })

            for edge in node.edges:
                if edge.child:
                    edges.append({
                        "source": node.id,
                        "target": edge.child.id,
                        "action": edge.action,
                        "visits": edge.visits,
                        "q": edge.q,
                        "prior": edge.prior
                    })
                    dfs(edge.child)

        dfs(root)

        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        with open(save_path, "w") as f:
            json.dump({"nodes": nodes, "edges": edges}, f, indent=2)

        print(f"Tree exported to {save_path}")

