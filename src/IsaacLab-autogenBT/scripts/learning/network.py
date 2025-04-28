###
#  Neural Network for Policy
###
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader

# ----------- Behavior Dictionary (char -> node type index) ----------- #
            # Flow Control
node_dict = { 0 : '0', # sequence_node
              1 : '1', # selector_node
              2 : '2', # parallel_node
            # Behaviors
              3 : 'a',   # patrol_node
              4 : 'b',   # find_target_node
              5 : 'c',   # go_to_nearest_target
              6 : 'd',   # go_to_charger_node
              7 : 'e',   # go_to_spawn_node
              8 : 'f',   # picking_object_node
              9 : 'g',   # drop_object_node
             10 : 'h',   # charge_node
            # Conditions
              11 : 'A',   # is_robot_at_the_charger_node
              12 : 'B',   # is_robot_at_the_spawn_node
              13 : 'C',   # is_battery_on_proper_level
              14 : 'D',   # are_object_existed_on_internal_map
              15 : 'E',   # are_object_nearby_node
              16 : 'F',   # is_object_in_hand_node
              17 : 'G',   # is_nearby_object_not_at_goal
              18 : 'H',   # are_five_objects_at_spawn
            # Specials
              19 : None, #stop node
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

class RvNN(nn.Module):
    """
    Recursive Neural Network for encoding Behavior Trees and predicting
    two discrete action probability distributions:
        1. Node type to expand
        2. Node location to apply expansion

    Args:
        node_type_vocab_size (int): Total number of node types (typically 20).
        embed_size (int): Size of the embedding vector for each node.
        hidden_size (int): Size of the hidden vector in recursive processing.
        action1_size (int): Output size for action 1 (node type prediction).
        action2_size (int): Output size for action 2 (location prediction).
    """
    def __init__(self, node_type_vocab_size, embed_size, hidden_size, action1_size, action2_size, device):
        super(RvNN, self).__init__()
        
        self.device = device
        
        self.node_embedding = nn.Embedding(node_type_vocab_size, embed_size, device=self.device)
        self.W_c = nn.Linear(embed_size + hidden_size, hidden_size, device=self.device)
        self.activation = nn.Tanh()

        self.output_action1 = nn.Linear(hidden_size, action1_size, device=self.device)
        self.output_action2 = nn.Linear(hidden_size, action2_size, device=self.device)

        self.child_gru = nn.GRU(hidden_size, hidden_size, batch_first=True)

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
        child_seq = torch.cat(child_states, dim=0).unsqueeze(0)  # shape: [1, num_children, hidden_size]

        _, last_hidden = self.child_gru(child_seq)  # shape: [1, 1, hidden_size]
        child_encoding = last_hidden.squeeze(0).squeeze(0)  # shape: [hidden_size]

        embed = self.node_embedding(torch.tensor([node.node_type], device=self.device))
        concat = torch.cat([embed.squeeze(0), child_encoding])  # [embed + child_hidden]
        h = self.activation(self.W_c(concat))
        return h

    def predict(self, bt_string):
        """
        Predict probability distributions for two actions from a BT string.

        Args:
            bt_string (str): A string representing a Behavior Tree (e.g., "(0a(1bC)").

        Returns:
            Tuple[Tensor, Tensor]: Probabilities for action1 (node type), action2 (location).
        """
        root = string_to_tree_node(bt_string)
        h = self.forward(root)

        action1_logits = self.output_action1(h)
        action2_logits = self.output_action2(h)

        action1_probs = torch.softmax(action1_logits, dim=0)
        action2_probs = torch.softmax(action2_logits, dim=0)

        return action1_probs, action2_probs
    
    def train_loop(self, train_loader, optimizer, num_epochs=10, val_loader=None, l2_weight=1e-4):
        """
        Train the RvNN model on the given dataset.

        Args:
            train_loader (DataLoader): DataLoader for training data.
            optimizer (torch.optim.Optimizer): Optimizer for training.
            num_epochs (int): Number of epochs to train.
            val_loader (DataLoader, optional): DataLoader for validation data. Default is None.
            l2_weight (float): Weight for L2 regularization. Default is 1e-4.

        Returns:
            None
        """
        for epoch in range(num_epochs):
            self.train()  # Enable dropout, batchnorm, etc.
            train_loss = 0.0
            train_correct1 = 0
            train_correct2 = 0

            for bt_strings, action1_targets, action2_targets in train_loader:
                for i, bt_string in enumerate(bt_strings):
                    optimizer.zero_grad()

                    # Predict logits
                    action1_logits, action2_logits = self.predict(bt_string)

                    # Convert one-hot target to index
                    target1 = action1_targets[i].argmax().unsqueeze(0)
                    target2 = action2_targets[i].argmax().unsqueeze(0)

                    # Compute loss
                    loss1 = self.loss_fn(action1_logits.unsqueeze(0), target1)
                    loss2 = self.loss_fn(action2_logits.unsqueeze(0), target2)

                    # L2 Regularization
                    l2_reg = sum((param**2).sum() for param in self.parameters())
                    loss = loss1 + loss2 + l2_weight * l2_reg

                    # Backward & optimize
                    loss.backward()
                    optimizer.step()

                    train_loss += loss.item()
                    train_correct1 += (action1_logits.argmax().item() == target1.item())
                    train_correct2 += (action2_logits.argmax().item() == target2.item())

            n_train = len(train_loader.dataset)
            avg_loss = train_loss / n_train
            acc1 = train_correct1 / n_train
            acc2 = train_correct2 / n_train
            print(f"Epoch {epoch+1}/{num_epochs} | Train Loss: {avg_loss:.4f} | Acc1: {acc1:.2%} | Acc2: {acc2:.2%}")

            # Validation if available
            if val_loader:
                self.eval()
                val_correct1 = 0
                val_correct2 = 0
                with torch.no_grad():
                    for bt_strings, action1_targets, action2_targets in val_loader:
                        for i, bt_string in enumerate(bt_strings):
                            action1_logits, action2_logits = self.predict(bt_string)
                            target1 = action1_targets[i].argmax()
                            target2 = action2_targets[i].argmax()
                            val_correct1 += (action1_logits.argmax().item() == target1.item())
                            val_correct2 += (action2_logits.argmax().item() == target2.item())

                n_val = len(val_loader.dataset)
                val_acc1 = val_correct1 / n_val
                val_acc2 = val_correct2 / n_val
                print(f"              | Val Acc1: {val_acc1:.2%} | Val Acc2: {val_acc2:.2%}")