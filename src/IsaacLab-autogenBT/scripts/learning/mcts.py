###
#  Monte-Carlo Tree Search
###
import json
import numpy as np
from tqdm import trange
import time
import os

def get_valid_action(bt_string, env, used_behavior_nodes):
    valid_locs_on_string = [j for j in range(1,len(bt_string)) if j == len(bt_string) or not bt_string[j].isdigit()] # Find valid location on BT string
    valid_locs = range(len(valid_locs_on_string))

    # [Experimental] Not allow adding a behavior node that is already used in the BT
    valid_nt = [nt for nt in range(1, env.num_node_types) if nt not in used_behavior_nodes]

    # [Experimental] Not allow above expansion if the root has only 1 child
    allow_above_expansion_flag = True
    
    num_nodes_in_first_depth = 0
    depth = 0
    for node in bt_string[1:-1]:
        if node == '(':
            if depth == 0:
                num_nodes_in_first_depth += 1
            depth += 1
        elif node == ')' :
            depth -= 1
        elif not node.isdigit() and depth == 0:
            num_nodes_in_first_depth += 1

    if num_nodes_in_first_depth == 1:
        allow_above_expansion_flag = False

    # Define all possible actions as (node_type, node_location) tuples
    # The node_location = 0 means the location of the parent node while the other n locations are the locations of the nth child node
    if bt_string == '':
        # If the BT is empty, only a location of the root node is valid
        all_actions = [(nt, 1) for nt in valid_nt]
    else:
        all_actions = allow_above_expansion_flag * [(nt, 0) for nt in range(4)] + [(nt, loc) for loc in range(1, len(valid_locs)) for nt in valid_nt]

    if sum(1 for c in bt_string if c not in ('(', ')')) >= env.nodes_limit:
        all_actions = []

    # If there are no valid locations, only the stop action is valid
    if (0, 0) not in all_actions:
        all_actions.append((0, 0))

    return all_actions

class MCTSNode:
    def __init__(self, state, env, policy_net, used_behavior_nodes = [], id = 0, parent_edge=None):
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
        self.value = None
        self.used_behavior_nodes = used_behavior_nodes
        self.evaluated_bt = None           # Store the BT used to evaluate this node (if modelfree)

        # Get all possible actions
        bt_string = state
        self.all_actions = get_valid_action(bt_string, self.env, self.used_behavior_nodes)

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
    def __init__(self, env, policy_net=None, num_simulations=50, num_pivots = 5, exploration_weight=1.0, fitness_mode = 'random', q_epsilon = 1e-3, model_based = False, device='cpu'):
        """
        Model-based Monte Carlo Tree Search with PUCT for Behavior Tree generation.

        :param env: MultiBTEnv environment instance.
        :param policy_net: Neural network to predict action probabilities.
        :param num_simulations: Number of simulations per search.
        :param exploration_weight: Weight for exploration in the PUCT formula.
        :param model_based: If True, use model-based evaluation.
        :param q_epsilon: Small constant to determine the information leaks.
        :param device: Compute device ('cpu' or 'cuda').
        """
        self.env = env
        self.num_simulations = num_simulations
        self.exploration_weight = exploration_weight
        self.device = device
        self.node_id = 0
        self.model_based = model_based
        self.transposition_nodes = []
        self.fitness_mode = fitness_mode
        self.q_epsilon = q_epsilon
        self.traj_branch_histories = [[] for _ in range(self.env.num_envs)]

        self.selected_depth = [0 for _ in range(self.env.num_envs)]
        self.node_depth_dict = {0: []}

        self.num_pivots = num_pivots

        if policy_net is not None:
            self.policy_net = policy_net.to(device)

    def run_search(self, root_state, temperature=1.0, dirichlet_noise_at_root=True, PUCT=True, verbose=False, export_path=None):
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
        self.transposition_nodes = []

        used_behavior_nodes_at_root = list(set(ch for ch in root_state if (not ch.isdigit()) and (ch not in ('(', ')'))))
        used_behavior_nodes_at_root = [k for k, v in self.env.node_dict.items() if v in used_behavior_nodes_at_root]
        root = MCTSNode(state=root_state, env=self.env, policy_net=self.policy_net, used_behavior_nodes=used_behavior_nodes_at_root)
        self.node_depth_dict = {0: [root]}
        self.non_terminated_node_list = []
        # >> print(f"Root possible actions: {root.all_actions}")

        pivots = [root for _ in range(self.env.num_envs)]
        self.selected_depth = [sum(1 for c in root.state if c not in ('(', ')')) for _ in range(self.env.num_envs)]

        if PUCT and self.policy_net is None:
            raise ValueError("PUCT requires a policy network to provide prior probabilities.")

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
            selected_edges = self.select(pivots, PUCT=PUCT, dirichlet_noise_at_root=dirichlet_noise_at_root)
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

            # Pivot new roots
            pivots = self.pivot(self.non_terminated_node_list, mode=self.fitness_mode)
            pivots = pivots + [root] * (self.env.num_envs - len(pivots))
            self.selected_depth = [sum(1 for c in pivot.state if c not in ('(', ')')) for pivot in pivots]
            # print(f'New pivots selected: {[node.state for node in pivots]}')

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

    def select(self, nodes, PUCT=True, dirichlet_noise_at_root=True):    
        """
        Selects edges using PUCT from the root to a leaf for each agent.

        :param nodes: List of root nodes (one per agent).
        :param dirichlet_noise_at_root: If True, applies noise to root node priors.
        :return: List of selected edges (one per agent).
        """
        epsilon = 0.25
        selected_edges = []
        
        root_nodes = nodes

        for env_id in range(self.env.num_envs):
            is_root_flag = dirichlet_noise_at_root
            node = root_nodes[env_id]
            while True:
                # Indicate the depth of the selected node
                self.selected_depth[env_id] += 1

                # Add Dirichlet noise at root node
                if is_root_flag:
                    actual_prior = []
                    dirichlet_noise = np.random.dirichlet([0.03] * len(node.edges))
                    for i, edge in enumerate(node.edges):
                        actual_prior.append(edge.prior)  # store original prior
                        edge.prior = float((1 - epsilon) * edge.prior + epsilon * dirichlet_noise[i])

                # Select the edge with the highest PUCT value
                node_visits = sum([edge.visits for edge in node.edges])
                if PUCT:
                    scores = np.array([
                                float(edge.q + self.exploration_weight * edge.prior * np.sqrt(node_visits) / (1 + edge.visits))
                                for edge in node.edges
                            ])
                else:
                    scores = np.array([
                                float(edge.q + self.exploration_weight * np.sqrt(node_visits) / (1 + edge.visits))
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

                # Check if next node is transposition node 
                if selected_edge.child in self.transposition_nodes:
                    self.traj_branch_histories[env_id].append(selected_edge)

                    # Determine the information leak (Q_delta)
                    Q_delta = selected_edge.child.value - selected_edge.q

                    if abs(Q_delta) > self.q_epsilon:
                        # Stop the selection and treat as leaf node to correct the information leak
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
            nt, loc = actions[env_id]
            used_behavior_nodes = edges[env_id].parent.used_behavior_nodes.copy()

            # Store the used behavior nodes
            if nt not in [0, 1, 2, 3]:
                used_behavior_nodes.append(nt)

            # Check if the child node is not already created
            if edges[env_id].child is None:
                # Find the index of the transposition node at the selected depth if exists
                try:
                    index = [node.state for node in self.node_depth_dict[self.selected_depth[env_id]]].index(obs[env_id])
                except:
                    index = None

                # Check if the node is transposition node
                if index is not None:
                    edges[env_id].child = self.node_depth_dict[self.selected_depth[env_id]][index]

                    # Update the parent edge of the transposition node
                    self.node_depth_dict[self.selected_depth[env_id]][index].parent_edge.append(edges[env_id])

                    # Record the transposition node in the list
                    self.transposition_nodes.append(self.node_depth_dict[self.selected_depth[env_id]][index])

                    self.traj_branch_histories[env_id].append(edges[env_id])

                else:
                    # Create a new child node for each selected edge if node is not transposition node 
                    self.node_id += 1
                    edges[env_id].child = MCTSNode(state=obs[env_id], 
                                                env=self.env, 
                                                policy_net=self.policy_net, 
                                                parent_edge=[edges[env_id]], 
                                                used_behavior_nodes=used_behavior_nodes, 
                                                id=self.node_id)

                    # Update node depth dictionary
                    if self.selected_depth[env_id] not in self.node_depth_dict.keys():
                        self.node_depth_dict[self.selected_depth[env_id]] = [edges[env_id].child]
                    else:
                        self.node_depth_dict[self.selected_depth[env_id]].append(edges[env_id].child)

                    if dones[env_id]:
                        edges[env_id].child.is_terminated = True
                    else:
                        self.non_terminated_node_list.append(edges[env_id].child)

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
        used_behavior_nodes = [node.used_behavior_nodes.copy() for node in nodes]

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
                    actions.append((0, 0))
                    continue

                # Get the action probablities from the policy network
                action_probs, _ = self.policy_net.predict(state) 

                # Convert torch tensors to numpy arrays
                action_probs = action_probs.detach().cpu().numpy()
  
                # Get all possible actions
                bt_string = state
                all_actions = get_valid_action(bt_string, self.env, used_behavior_nodes[env_id])

                # Select the best action according to its joint probability
                probs = []
                for nt, loc in all_actions:
                    if loc == 0:
                        probs.append(float(action_probs[nt]))
                    else:
                        probs.append(float(action_probs[3 + nt + (self.env.num_node_types - 1) * (loc - 1)]))

                best_ind = np.where(probs == np.max(probs))[0]
                selected_ind = np.random.choice(best_ind)

                nt, loc = all_actions[selected_ind]

                # Store the used behavior nodes
                if nt not in [0, 1, 2, 3]:
                    used_behavior_nodes[env_id].append(nt)

                actions.append(all_actions[selected_ind])

            # Perform the action in the environment (add node to BT)
            obs, _, dones, infos =  self.env.step_without_sim(actions) 

            # Update the state for each node
            states = obs

            # Check if the BTs construction is done
            if all(dones):
                break

        # print(f"[INFO] \tBTs constructed: {states}")

        # Store the evaluated BT strings in the nodes
        for env_id in range(self.env.num_envs):
            nodes[env_id].evaluated_bt = states[env_id]

        # Get the reward by runnung the BT in IsaacSim Simulation
        _, rews, _, infos =  self.env.evaluate_bt_in_sim()

        # Update the value for each node
        for env_id in range(self.env.num_envs):
            nodes[env_id].value = rews[env_id]

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

            # Update the value for each node
            nodes[env_id].value = rew

        return rews
    
    def pivot(self, nodes, mode='random'):

        if mode == 'random':
            ind = self._random_fitness(nodes)
        elif mode == 'less_nodes':
            ind = self._less_nodes_fitness(nodes)

        pivots = []
        for i in ind[:self.num_pivots]:
            pivots.append(nodes[i])

        pivots = pivots * (self.env.num_envs // self.num_pivots)

        return pivots
    
    def _random_fitness(self, nodes):
        values = np.array([node.value for node in nodes])

        # Generate a random key to break ties
        rand_key = np.random.random(len(nodes))

        # Sort by: (1) values descending, (2) random key
        ind = np.lexsort((rand_key, -values))

        return ind
    
    def _less_nodes_fitness(self, nodes):
        values = np.array([node.value for node in nodes])
        num_nodes = [sum(1 for c in node.state if c not in ('(', ')')) for node in nodes]

        # Sort by: (1) values descending, (2) numnode ascending
        ind = np.lexsort((num_nodes, -values))

        return ind

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
                # If node.parent_edge is None, we reached the root
                if node.parent_edge is None:
                    break
                
                if len(node.parent_edge) > 1:
                    # If the node has multiple parent edges (transposition node), update only the traversed edge
                    try:
                        traversed_edge = self.traj_branch_histories[env_id].pop()
                    except:
                        traversed_edge = node.parent_edge[0]
                else:
                    # Find the traversed edge
                    traversed_edge = node.parent_edge[0]

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
                "value": node.value,
                "evaluated_bt": node.evaluated_bt,
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

