###
#  Neural Network for Policy
###
import torch
import torch.nn as nn
from cachetools import LRUCache

# ----------- Behavior Dictionary (char -> node type index) ----------- #
#             # Flow Control
# node_dict = { 0 : '0', # sequence_node
#               1 : '1', # selector_node
#               2 : '2', # parallel_node
#             # Behaviors
#               3 : 'a',   # patrol_node
#               4 : 'b',   # find_target_node
#               5 : 'c',   # go_to_nearest_target
#               6 : 'd',   # go_to_charger_node
#               7 : 'e',   # go_to_spawn_node
#               8 : 'f',   # picking_object_node
#               9 : 'g',   # drop_object_node
#              10 : 'h',   # charge_node
#             # Conditions
#               11 : 'A',   # is_robot_at_the_charger_node
#               12 : 'B',   # is_robot_at_the_spawn_node
#               13 : 'C',   # is_battery_on_proper_level
#               14 : 'D',   # are_object_existed_on_internal_map
#               15 : 'E',   # are_object_nearby_node
#               16 : 'F',   # is_object_in_hand_node
#               17 : 'G',   # is_nearby_object_not_at_goal
#               18 : 'H',   # are_five_objects_at_spawn
#             # Specials
#               19 : None, #stop node
#             }

# node_dict = {   0 : '0', #patrol_node
#                 1 : '1', #find_target_node
#                 2 : '2', #go_to_nearest_target
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
#                 13 : 'G', #is_nearby_object_not_at_goal
#                 14 : 'H', #are_five_objects_at_spawn
#                 # Specials
#                 15 : None, #stop node
#                 }

node_dict = {   0 : '0', #patrol_node
                1 : '1', #find_target_node
                2 : '2', #go_to_nearest_target
                # Behaviors
                3 : 'a', #patrol_node
                4 : 'b', #find_target_node
                5 : 'c', #go_to_nearest_target
                # Conditions
                6 : 'D', #are_object_existed_on_internal_map
                # Specials
                7 : None, #stop node
                }

# === Inverted character -> type index ===
char_to_node_type = {v: k for k, v in node_dict.items() if v is not None}

# === Composite node types ===
composite_chars = {'0', '1', '2'}

class TreeNode:
    """
    Simple recursive tree structure to represent a Behavior Tree (BT) node.
    
    Args:
        name (str): Node name for readability.
        node_type (int): Index representing the type of node (from node_dict).
        children (list): List of child TreeNode objects.
    """
    def __init__(self, name, node_type, children=None):
        self.name = name
        self.node_type = node_type
        self.children = children if children else []

def string_to_tree_node(tree_string, node_id=0):
    """
    Parse a BT string representation directly into a TreeNode-based tree.
    
    Args:
        tree_string (str): A BT string like "(0a(1bC))".
        node_id (int): Used for naming composite nodes uniquely (optional).
    
    Returns:
        TreeNode: Root of the parsed tree.
    """
    # Detect empty tree
    if tree_string == '':
        return TreeNode("emty_tree", 19)
    
    # Detect a single node tree
    if len(tree_string) == 1:
        node_type = char_to_node_type[tree_string[0]]
        name = f"leaf_{tree_string[0]}"
        return TreeNode(name, node_type)

    # First character after '(' is the root node type
    node_type_char = tree_string[1]
    
    if node_type_char in composite_chars:
        # It's a flow control (composite) node
        node_type = char_to_node_type[node_type_char]
        name = f"composite_{node_id}"
        node_id += 1
    elif node_type_char in char_to_node_type:
        # It's a single behavior/condition node
        node_type = char_to_node_type[node_type_char]
        name = f"leaf_{node_type_char}"
        return TreeNode(name, node_type)
    else:
        raise ValueError(f"[Error] Unknown node type char: {node_type_char}")

    # Recursively parse children
    children = []
    i = 2
    while i < len(tree_string) - 1:
        c = tree_string[i]
        if c == '(':
            # Start of a subtree
            depth = 1
            start = i
            while depth != 0:
                i += 1
                if tree_string[i] == '(':
                    depth += 1
                elif tree_string[i] == ')':
                    depth -= 1
            subtree = tree_string[start:i+1]
            child = string_to_tree_node(subtree, node_id)
            children.append(child)
            i += 1
        elif c in char_to_node_type:
            # Leaf node (single character)
            child = TreeNode(f"leaf_{c}", char_to_node_type[c])
            children.append(child)
            i += 1
        else:
            print("[Error] Skipping unknown char:", c)
            i += 1

    return TreeNode(name, node_type, children)

class RvNN_mem(nn.Module):
    """
    Recursive Neural Network for encoding Behavior Trees and predicting:
        1. Node type to expand (classification)
        2. Node location to apply expansion (classification)
        3. Scalar reward (regression)
    """
    def __init__(self, node_type_vocab_size, embed_size, hidden_size, action1_size, action2_size, device, reward_head = False, cache_size=10000, parse_cache_size=10000):
        super(RvNN_mem, self).__init__()
        
        self.device = device
        
        self.node_embedding = nn.Embedding(node_type_vocab_size, embed_size, device=self.device)
        self.W_c = nn.Linear(embed_size + hidden_size, hidden_size, device=self.device)
        self.activation = nn.Tanh()

        self.output_action1 = nn.Linear(hidden_size, action1_size, device=self.device)
        self.output_action2 = nn.Linear(hidden_size, action2_size, device=self.device)

        self.reward_head = None
        if reward_head:
            self.reward_head = nn.Linear(hidden_size, 1, device=self.device)

        self.child_gru = nn.GRU(hidden_size, hidden_size, batch_first=True, device=self.device)

        self.loss_fn = nn.CrossEntropyLoss()
        self.loss_fn_reward = nn.MSELoss()

        # LRU Cache for memoization: key is the subtree string
        self.memo = LRUCache(maxsize=cache_size)
        self.tree_parse_memo = LRUCache(maxsize=parse_cache_size)  # parsed TreeNode cache

        self.cache_hits = 0
        self.cache_misses = 0

    def _parse_tree_node(self, tree_string):
        """
        Memoized version of string_to_tree_node.
        """
        if tree_string in self.tree_parse_memo:
            return self.tree_parse_memo[tree_string]
        node = string_to_tree_node(tree_string)
        self.tree_parse_memo[tree_string] = node
        return node

    def forward(self, bt_string):
        """
        Forward pass: encode a Behavior Tree string recursively.

        Args:
            bt_string (str): Behavior Tree string.

        Returns:
            torch.Tensor: Hidden vector encoding of the tree.
        """
        return self._encode_node(bt_string)

    def _encode_node(self, tree_string):
        """
        Recursively encode a subtree from its string representation.

        Args:
            tree_string (str): Subtree string.

        Returns:
            torch.Tensor: Encoded hidden state.
        """
        if tree_string in self.memo:
            self.cache_hits += 1
            return self.memo[tree_string].detach().clone().requires_grad_()
        
        self.cache_misses += 1  

        node = self._parse_tree_node(tree_string)
        # node = string_to_tree_node(tree_string)

        if not node.children:
            embed = self.node_embedding(torch.tensor([node.node_type], device=self.device))
            zero_pad = torch.zeros(self.W_c.in_features - embed.size(1), device=self.device)
            h = self.activation(self.W_c(torch.cat([embed.squeeze(0), zero_pad])))
            self.memo[tree_string] = h
            return h

        # recursively process each child string
        child_substrings = self._extract_child_substrings(tree_string)
        child_states = [self._encode_node(subtree).unsqueeze(0) for subtree in child_substrings]
        child_seq = torch.cat(child_states, dim=0).unsqueeze(0)

        _, last_hidden = self.child_gru(child_seq)
        child_encoding = last_hidden.squeeze(0).squeeze(0)

        embed = self.node_embedding(torch.tensor([node.node_type], device=self.device))
        concat = torch.cat([embed.squeeze(0), child_encoding])
        h = self.activation(self.W_c(concat))

        self.memo[tree_string] = h
        return h

    def _extract_child_substrings(self, tree_string):
        """
        Parse and extract immediate child subtree strings from a parent string.

        Example:
            "(0a(1bC))" → ['a', '(1bC)']

        Args:
            tree_string (str): Parent tree string.

        Returns:
            list of str: List of child subtree strings.
        """
        children = []
        i = 2  # skip '(' and node type char
        while i < len(tree_string) - 1:
            c = tree_string[i]
            if c == '(':
                # parse nested subtree
                depth = 1
                start = i
                while depth != 0:
                    i += 1
                    if tree_string[i] == '(':
                        depth += 1
                    elif tree_string[i] == ')':
                        depth -= 1
                subtree = tree_string[start:i+1]
                children.append(subtree)
                i += 1
            elif c in char_to_node_type:
                children.append(c)
                i += 1
            else:
                print("[Warning] Unknown character during child extraction:", c)
                i += 1
        return children

    def predict(self, bt_string):
        """
        Predict two action probability distributions and scalar reward.

        Args:
            bt_string (str): A string representing a Behavior Tree.

        Returns:
            Tuple[Tensor, Tensor, Tensor]: 
                - Probabilities for action1 (node type)
                - Probabilities for action2 (location)
                - Scalar reward prediction
        """
        torch.autograd.set_detect_anomaly(True)
        h = self.forward(bt_string)

        action1_logits = self.output_action1(h)
        action2_logits = self.output_action2(h)
        if self.reward_head is not None: reward_pred = self.reward_head(h).squeeze(0) 
        else: reward_pred = None

        action1_probs = torch.softmax(action1_logits, dim=0)
        action2_probs = torch.softmax(action2_logits, dim=0)

        return action1_probs, action2_probs, reward_pred

    def train_loop(self, train_loader, optimizer, num_epochs=10, val_loader=None, l2_weight=1e-4, writer=None, global_step=0):
        """
        Train the RvNN model on the given dataset.

        Args:
            train_loader (DataLoader): DataLoader with (bt_string, action1, action2, reward).
            optimizer (torch.optim.Optimizer): Optimizer to use.
            num_epochs (int): Number of training epochs.
            val_loader (DataLoader, optional): Validation DataLoader.
            l2_weight (float): L2 regularization weight.
        """
        for epoch in range(num_epochs):
            self.train()
            train_loss = 0.0
            train_correct1 = 0
            train_correct2 = 0
            reward_mse_total = 0.0

            for bt_strings, action1_targets, action2_targets, reward_targets in train_loader:
                for i, bt_string in enumerate(bt_strings):
                    optimizer.zero_grad()

                    action1_logits, action2_logits, reward_pred = self.predict(bt_string)

                    target1 = action1_targets[i].argmax().unsqueeze(0)
                    target2 = action2_targets[i].argmax().unsqueeze(0)
                    reward_target = reward_targets[i].float().to(self.device)

                    loss1 = self.loss_fn(action1_logits.unsqueeze(0), target1)
                    loss2 = self.loss_fn(action2_logits.unsqueeze(0), target2)
                    if self.reward_head is not None: loss3 = self.loss_fn_reward(reward_pred, reward_target)
                    else: loss3 = 0

                    l2_reg = sum((param**2).sum() for param in self.parameters())
                    loss = loss1 + loss2 + loss3 + l2_weight * l2_reg

                    torch.autograd.set_detect_anomaly(True)
                    loss.backward()
                    optimizer.step()

                    train_loss += loss.item()
                    train_correct1 += (action1_logits.argmax().item() == target1.item())
                    train_correct2 += (action2_logits.argmax().item() == target2.item())
                    if self.reward_head is not None: reward_mse_total += (reward_pred.item() - reward_target.item()) ** 2

            n_train = len(train_loader.dataset)
            avg_loss = train_loss / n_train
            acc1 = train_correct1 / n_train
            acc2 = train_correct2 / n_train

            if self.reward_head is not None:
                reward_mse = reward_mse_total / n_train
                print(f"Epoch {epoch+1}/{num_epochs} | Train Loss: {avg_loss:.4f} | Acc1: {acc1:.2%} | Acc2: {acc2:.2%} | Reward MSE: {reward_mse:.4f}")
            else:
                print(f"Epoch {epoch+1}/{num_epochs} | Train Loss: {avg_loss:.4f} | Acc1: {acc1:.2%} | Acc2: {acc2:.2%}")

            if writer:
                writer.add_scalar("Train/Loss", avg_loss, global_step + epoch)
                writer.add_scalar("Train/Acc1_NodeType", acc1, global_step + epoch)
                writer.add_scalar("Train/Acc2_Location", acc2, global_step + epoch)
                if self.reward_head is not None: writer.add_scalar("Train/Reward_MSE", reward_mse, global_step + epoch)

            # Validation
            if val_loader:
                self.eval()
                val_correct1 = 0
                val_correct2 = 0
                val_reward_mse = 0.0

                with torch.no_grad():
                    for bt_strings, action1_targets, action2_targets, reward_targets in val_loader:
                        for i, bt_string in enumerate(bt_strings):
                            action1_logits, action2_logits, reward_pred = self.predict(bt_string)

                            target1 = action1_targets[i].argmax()
                            target2 = action2_targets[i].argmax()
                            reward_target = reward_targets[i].float().to(self.device)

                            val_correct1 += (action1_logits.argmax().item() == target1.item())
                            val_correct2 += (action2_logits.argmax().item() == target2.item())
                            if self.reward_head is not None: val_reward_mse += (reward_pred.item() - reward_target.item()) ** 2

                n_val = len(val_loader.dataset)
                val_acc1 = val_correct1 / n_val
                val_acc2 = val_correct2 / n_val

                if self.reward_head is not None:
                    val_reward_mse /= n_val
                    print(f"              | Val Acc1: {val_acc1:.2%} | Val Acc2: {val_acc2:.2%} | Val Reward MSE: {val_reward_mse:.4f}")
                else:
                    print(f"              | Val Acc1: {val_acc1:.2%} | Val Acc2: {val_acc2:.2%}")

                if writer:
                    writer.add_scalar("Val/Acc1_NodeType", val_acc1, global_step + epoch)
                    writer.add_scalar("Val/Acc2_Location", val_acc2, global_step + epoch)
                    if self.reward_head is not None: writer.add_scalar("Val/Reward_MSE", val_reward_mse, global_step + epoch)

        print(f"Cache hits: {self.cache_hits} | Cache misses: {self.cache_misses} | Hit ratio: {100 * self.cache_hits / (self.cache_hits + self.cache_misses + 1e-8):.2f}%")
        return avg_loss  # Return the final average loss for outer-loop logging

class RvNN(nn.Module):
    """
    Recursive Neural Network for encoding Behavior Trees and predicting:
        1. Node type to expand (classification)
        2. Node location to apply expansion (classification)
        3. Scalar reward (regression)
    """
    def __init__(self, node_type_vocab_size, embed_size, hidden_size, action1_size, action2_size, device, reward_head=False):
        super(RvNN, self).__init__()
        
        self.device = device
        
        self.node_embedding = nn.Embedding(node_type_vocab_size, embed_size, device=self.device)
        self.W_c = nn.Linear(embed_size + hidden_size, hidden_size, device=self.device)
        self.activation = nn.Tanh()

        self.output_action1 = nn.Linear(hidden_size, action1_size, device=self.device)
        self.output_action2 = nn.Linear(hidden_size, action2_size, device=self.device)

        self.reward_head = None
        if reward_head:
            self.reward_head = nn.Linear(hidden_size, 1, device=self.device)

        self.child_gru = nn.GRU(hidden_size, hidden_size, batch_first=True, device=self.device)

        self.loss_fn = nn.CrossEntropyLoss()
        self.loss_fn_reward = nn.MSELoss()

    def forward(self, node):
        """
        Forward pass: encode a TreeNode structure recursively.

        Args:
            node (TreeNode): Root node of the tree.

        Returns:
            torch.Tensor: Hidden vector encoding of the tree.
        """
        return self._encode_node(node)

    def _encode_node(self, node):
        """
        Internal method to recursively encode a TreeNode using a child-GRU.

        Args:
            node (TreeNode): A node to process.

        Returns:
            torch.Tensor: Encoded hidden state for this subtree.
        """
        if not node.children:
            embed = self.node_embedding(torch.tensor([node.node_type], device=self.device))
            zero_pad = torch.zeros(self.W_c.in_features - embed.size(1), device=self.device)
            h = self.activation(self.W_c(torch.cat([embed.squeeze(0), zero_pad])))
            return h

        # Encode children recursively
        child_states = [self._encode_node(child).unsqueeze(0) for child in node.children]
        child_seq = torch.cat(child_states, dim=0).unsqueeze(0)  # [1, num_children, hidden_size]

        _, last_hidden = self.child_gru(child_seq)  # [1, 1, hidden_size]
        child_encoding = last_hidden.squeeze(0).squeeze(0)

        embed = self.node_embedding(torch.tensor([node.node_type], device=self.device))
        concat = torch.cat([embed.squeeze(0), child_encoding])  # [embed + child_hidden]
        h = self.activation(self.W_c(concat))
        return h

    def predict(self, bt_string):
        """
        Predict two action probability distributions and scalar reward.

        Args:
            bt_string (str): A string representing a Behavior Tree.

        Returns:
            Tuple[Tensor, Tensor, Tensor]: 
                - Probabilities for action1 (node type)
                - Probabilities for action2 (location)
                - Scalar reward prediction
        """
        root = string_to_tree_node(bt_string)
        h = self.forward(root)

        action1_logits = self.output_action1(h)
        action2_logits = self.output_action2(h)
        if self.reward_head is not None: reward_pred = self.reward_head(h).squeeze(0) 
        else: reward_pred = None

        action1_probs = torch.softmax(action1_logits, dim=0)
        action2_probs = torch.softmax(action2_logits, dim=0)

        return action1_probs, action2_probs, reward_pred

    def train_loop(self, train_loader, optimizer, num_epochs=10, val_loader=None, l2_weight=1e-4, writer=None, global_step=0):
        """
        Train the RvNN model on the given dataset.

        Args:
            train_loader (DataLoader): DataLoader with (bt_string, action1, action2, reward).
            optimizer (torch.optim.Optimizer): Optimizer to use.
            num_epochs (int): Number of training epochs.
            val_loader (DataLoader, optional): Validation DataLoader.
            l2_weight (float): L2 regularization weight.
        """
        for epoch in range(num_epochs):
            self.train()
            train_loss = 0.0
            train_correct1 = 0
            train_correct2 = 0
            reward_mse_total = 0.0

            for bt_strings, action1_targets, action2_targets, reward_targets in train_loader:
                for i, bt_string in enumerate(bt_strings):
                    optimizer.zero_grad()

                    action1_logits, action2_logits, reward_pred = self.predict(bt_string)

                    target1 = action1_targets[i].argmax().unsqueeze(0)
                    target2 = action2_targets[i].argmax().unsqueeze(0)
                    reward_target = reward_targets[i].float().to(self.device)

                    loss1 = self.loss_fn(action1_logits.unsqueeze(0), target1)
                    loss2 = self.loss_fn(action2_logits.unsqueeze(0), target2)
                    if self.reward_head is not None: loss3 = self.loss_fn_reward(reward_pred, reward_target)
                    else: loss3 = 0

                    l2_reg = sum((param**2).sum() for param in self.parameters())
                    loss = loss1 + loss2 + loss3 + l2_weight * l2_reg

                    loss.backward()
                    optimizer.step()

                    train_loss += loss.item()
                    train_correct1 += (action1_logits.argmax().item() == target1.item())
                    train_correct2 += (action2_logits.argmax().item() == target2.item())
                    if self.reward_head is not None: reward_mse_total += (reward_pred.item() - reward_target.item()) ** 2

            n_train = len(train_loader.dataset)
            avg_loss = train_loss / n_train
            acc1 = train_correct1 / n_train
            acc2 = train_correct2 / n_train

            if self.reward_head is not None:
                reward_mse = reward_mse_total / n_train
                print(f"Epoch {epoch+1}/{num_epochs} | Train Loss: {avg_loss:.4f} | Acc1: {acc1:.2%} | Acc2: {acc2:.2%} | Reward MSE: {reward_mse:.4f}")
            else:
                print(f"Epoch {epoch+1}/{num_epochs} | Train Loss: {avg_loss:.4f} | Acc1: {acc1:.2%} | Acc2: {acc2:.2%}")

            if writer:
                writer.add_scalar("Train/Loss", avg_loss, global_step + epoch)
                writer.add_scalar("Train/Acc1_NodeType", acc1, global_step + epoch)
                writer.add_scalar("Train/Acc2_Location", acc2, global_step + epoch)
                if self.reward_head is not None: writer.add_scalar("Train/Reward_MSE", reward_mse, global_step + epoch)

            # Validation
            if val_loader:
                self.eval()
                val_correct1 = 0
                val_correct2 = 0
                val_reward_mse = 0.0

                with torch.no_grad():
                    for bt_strings, action1_targets, action2_targets, reward_targets in val_loader:
                        for i, bt_string in enumerate(bt_strings):
                            action1_logits, action2_logits, reward_pred = self.predict(bt_string)

                            target1 = action1_targets[i].argmax()
                            target2 = action2_targets[i].argmax()
                            reward_target = reward_targets[i].float().to(self.device)

                            val_correct1 += (action1_logits.argmax().item() == target1.item())
                            val_correct2 += (action2_logits.argmax().item() == target2.item())
                            if self.reward_head is not None: val_reward_mse += (reward_pred.item() - reward_target.item()) ** 2

                n_val = len(val_loader.dataset)
                val_acc1 = val_correct1 / n_val
                val_acc2 = val_correct2 / n_val

                if self.reward_head is not None:
                    val_reward_mse /= n_val
                    print(f"              | Val Acc1: {val_acc1:.2%} | Val Acc2: {val_acc2:.2%} | Val Reward MSE: {val_reward_mse:.4f}")
                else:
                    print(f"              | Val Acc1: {val_acc1:.2%} | Val Acc2: {val_acc2:.2%}")

                if writer:
                    writer.add_scalar("Val/Acc1_NodeType", val_acc1, global_step + epoch)
                    writer.add_scalar("Val/Acc2_Location", val_acc2, global_step + epoch)
                    if self.reward_head is not None: writer.add_scalar("Val/Reward_MSE", val_reward_mse, global_step + epoch)

        return avg_loss  # Return the final average loss for outer-loop logging