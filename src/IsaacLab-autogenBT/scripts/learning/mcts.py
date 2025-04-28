###
#  Monte-Carlo Tree Search
###
import numpy as np
import random
import math

class MCTSNode:
    def __init__(self, state, env, policy_net, parent_edge=None):
        """
        Represents a single state (node) in the MCTS tree for Behavior Tree construction.

        :param state: String representation of the current Behavior Tree (BT).
        :param env: The MultiBTEnv environment instance.
        :param policy_net: Neural network (RvNN) that outputs action probabilities.
        :param parent_edge: The edge from the parent node leading to this node.
        """
        self.state = state                  # BT string for this node
        self.env = env                      # OpenGym Environment
        self.policy_net = policy_net        # RvNN model for policy and value
        self.is_terminated = False           # Flag to indicate if this node is terminal
        self.parent_edge = parent_edge                # Parent node

        # Get all possible actions
        bt_string = state[1:-1] if state.startswith('(') else state
        valid_locs_on_string = [j for j in range(len(bt_string)+1) if j == len(bt_string) or not bt_string[j].isdigit()] # Find valid location on BT string
        valid_locs = range(len(valid_locs_on_string))
        self.all_actions = [(nt, loc) for nt in range(self.env.num_node_types) for loc in range(len(valid_locs))]

        # Get prior porbabilities from the policy network
        nt_probs, loc_probs = self.policy_net.predict(state) 

        # Initialize edges for each action
        self.edges = [MCTSEdge(self, (nt, loc), (nt_probs[nt] * loc_probs[loc]).item()) for nt, loc in self.all_actions]
    
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
    def __init__(self, env, policy_net, num_simulations=50, exploration_weight=1.0, device='cpu', verbose=False):
        """
        Monte Carlo Tree Search with PUCT for Behavior Tree generation.

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
        self.verbose = verbose

    def run_search(self, root_state, temperature=1.0):
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

        for i in range(self.num_simulations):
            if self.verbose: print(f"[INFO] MCTS iteration {i}...")

            # Select a leaf node
            selected_edges = self.select(root, dirichlet_noise_at_root=True)

            # Expand the selected leaf node
            self.expand(selected_edges)
            leave_nodes = [edge.child for edge in selected_edges]

            # Evaluate the expanded nodes
            rewards = self.evaluate(leave_nodes)

            # Backpropagate the results
            self.backpropagate(leave_nodes, rewards)

        # Collect visit counts and probabilities
        nt_probs = np.zeros(self.env.num_node_types)
        loc_probs = np.zeros(self.env.max_location_size)

        for edge in root.edges:
            nt, loc = edge.action
            nt_probs[nt] += edge.visits
            loc_probs[loc] += edge.visits

        nt_probs /= np.sum(nt_probs)
        loc_probs /= np.sum(loc_probs)

        nt_probs = nt_probs ** (1.0 / temperature)
        loc_probs = loc_probs ** (1.0 / temperature)

        return nt_probs, loc_probs

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
                    # Add Dirichlet noise to the prior probabilities
                    dirichlet_noise = np.random.dirichlet([0.03] * len(node.edges))
                    for i, edge in enumerate(node.edges):
                        actual_prior.append(edge.prior) # Store the original prior
                        edge.prior = (1 - epsilon) * edge.prior + epsilon * dirichlet_noise[i]

                # Select the edge with the highest PUCT value
                node_visits = sum([edge.visits for edge in node.edges])
                scores = np.array([edge.q + self.exploration_weight * edge.prior * np.sqrt(node_visits) / (1 + edge.visits)
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
        Expands leaf nodes by performing their selected actprint(edges[env_id].parent.state)ons and creating children.

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
                edges[env_id].child = MCTSNode(state=obs[env_id], env=self.env, policy_net=self.policy_net, parent_edge=edges[env_id])
                
                if dones[env_id]:
                    edges[env_id].child.is_terminated = True

    def evaluate(self, nodes):
        """
        Evaluates completed BTs using the full IsaacSim simulation.

        :param nodes: Nodes with fully expanded BTs.
        :return: List of simulation-derived rewards.
        """
        states = [node.state for node in nodes]
        dones = [False for _ in range(self.env.num_envs)]

        # Make sure that the BT string is matched with the state in leaf node
        for env_id in range(self.env.num_envs):
            self.env.set_bt(env_id=env_id, bt_string=states[env_id])
            
        # Expand the BT until it reaches a terminal state (stop action or max depth)
        while True:
            actions = []
            for env_id in range(self.env.num_envs):
                state = states[env_id]
                
                # If the BT Contruction is done, select stop action
                if nodes[env_id].is_terminated or dones[env_id]:
                    actions.append((19, 0))
                    continue

                # Get the action probablities from the policy network
                nt_probs, loc_probs = self.policy_net.predict(state) 

                # Convert torch tensors to numpy arrays
                nt_probs = nt_probs.detach().cpu().numpy()
                loc_probs = loc_probs.detach().cpu().numpy()

                # Mask to get only valid node locations
                bt_string = state[1:-1] if state.startswith('(') else state
                valid_locs_on_string = [j for j in range(len(bt_string)+1) if j == len(bt_string) or not bt_string[j].isdigit()] # Find valid location on BT string
                loc_probs = loc_probs[:len(valid_locs_on_string)]

                max_nt_probs = np.max(nt_probs)
                max_loc_probs = np.max(loc_probs)

                best_nt_indices = np.where(nt_probs == max_nt_probs)[0]
                best_loc_indices = np.where(loc_probs == max_loc_probs)[0]

                # Randomly select one of the best indices
                selected_nt = np.random.choice(best_nt_indices)
                selected_loc = np.random.choice(best_loc_indices)

                actions.append((selected_nt, selected_loc))

            # Perform the action in the environment (add node to BT)
            obs, _, dones, infos =  self.env.step_without_sim(actions) 

            # Update the state for each node
            states = obs

            # Check if the BTs construction is done
            if all(dones):
                break

        # Get the reward by runnung the BT in IsaacSim Simulation
        if self.verbose: print(f"[INFO] \tEvaluating {self.env.current_bt}")
        _, rews, _, infos =  self.env.evaluate_bt_in_sim()
        if self.verbose: print(f"[INFO] \tFinished Evaluation") 

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