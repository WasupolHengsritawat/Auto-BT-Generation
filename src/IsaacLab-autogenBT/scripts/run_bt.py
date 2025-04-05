import os
import sys
import math
import threading
import networkx as nx
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees.console as console
from py_trees_ros.trees import BehaviourTree
from geometry_msgs.msg import Point, Twist
import argparse
import subprocess

parser = argparse.ArgumentParser(description="Run your script with arguments.")
parser.add_argument('--env_id', type=int, help="Environment ID of the behavior tree.")
parser.add_argument('--bt_string', type=str, help="Behavior tree in string format.")
parser.add_argument('--verbose', type=bool, default=False, help="Display behavior tree execution status in terminal.")

args_cli = parser.parse_args()

# Import behavior nodes
cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT/scripts')
from behavior import (
    PatrolNode, MoveNode, GoToChargerNode, FindTargetNode, 
    AreObjectsExistOnInternalMap, GoToNearestTarget, AreObjectNearby, 
    PickObject, Charge, IsRobotAtTheCharger, IsRobotAtTheSpawn, 
    IsBatteryOnProperLevel, IsObjectInHand, DropObject, GoToSpawnNode, 
    IsNearbyObjectNotAtGoal, AreFiveObjectsAtSpawn
)

def euclidean_distance(coord1, coord2):
    """
    Calculate the Euclidean distance between two coordinates.
    
    Args:
        coord1 (tuple): (x, y) coordinates of the first point.
        coord2 (tuple): (x, y) coordinates of the second point.
    
    Returns:
        float: The Euclidean distance between coord1 and coord2.
    """
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def graph_init(offset):
    """
    Initialize the graph representation of the cave map.
    
    Args:
        offset (tuple): (x, y) offset to apply to all coordinates.
    
    Returns:
        tuple: Three NetworkX graphs (target_graph, full_graph, robot_graph) representing different aspects of the environment.
    """
    coordinates = [(  0.00,  0.00), #  0 - spawn point
                   ( -2.00,  0.17), #  1 - charging station
                   ( 10.30,  2.00), #  2
                   ( 14.00, -1.10), #  3
                   ( 19.40,  3.60), #  4
                   ( 15.24,  6.60), #  5
                   ( 11.50,  7.55), #  6
                   (  7.74, 11.00), #  7
                   (  1.17,  7.91), #  8
                   (  0.72, 13.64), #  9
                   (  2.80, 17.74), # 10
                   (  8.08, 15.50), # 11
                   ( 10.72, 20.10), # 12
                   ( 12.63, 22.51), # 13
                   ( 15.44, 23.41), # 14
                   ( 18.19, 21.00), # 15
                   ( 21.10, 19.60), # 16
                   ( 19.80, 14.94), # 17
                   ( 17.95, 15.11), # 18
                   ( 15.93, 17.75), # 19
                   ( 14.00, -4.00), # 20 - target  #1
                   ( 22.00, -1.00), # 21 - target  #2
                   ( 19.50,  7.70), # 22 - target  #3
                   ( 20.50, 13.00), # 23 - target  #4
                   ( 23.00, 21.00), # 24 - target  #5
                   (  9.00, 24.50), # 25 - target  #6
                   ( -2.00, 23.20), # 26 - target  #7
                   ( -1.50, 13.30), # 27 - target  #8
                   ( -2.60,  7.70), # 28 - target  #9
                   (  8.00,  6.70)] # 29 - target #10
    
    # Apply offset to all coordinates
    coordinates = [(x + offset[0], y + offset[1]) for (x, y) in coordinates]
    
    # Connection between nodes
    connections = [( 0, 1),
                   ( 0, 2),
                   ( 2, 3),
                   ( 2, 6),
                   ( 3, 4),
                   ( 4, 5),
                   ( 5, 6),
                   ( 6, 7),
                   ( 7, 8),
                   ( 7,11),
                   ( 8, 9),
                   ( 9,10),
                   (10,11),
                   (11,12),
                   (12,13),
                   (12,19),
                   (13,14),
                   (14,15),
                   (15,16),
                   (15,19),
                   (16,17),
                   (17,18),
                   (18,19),
                   ( 3,20),
                   ( 4,21),
                   ( 4,22),
                   ( 5,22),
                   (17,23),
                   (16,24),
                   (13,25),
                   (10,26),
                   ( 9,27),
                   ( 8,28),
                   ( 7,29),
                   ( 6,29)]
    
    # Create graphs 
    full_graph, robot_graph, target_graph = nx.Graph(), nx.Graph(), nx.Graph()

    # Add nodes
    for i, coord in enumerate(coordinates):
        target_graph.add_node(i, pos=coord)
        if i <= 19:
            full_graph.add_node(i, pos=coord)
        if i <= 1:
            robot_graph.add_node(i, pos=coord)

    # Add edges with weights as Euclidean distances
    for (u, v) in connections:
        distance = euclidean_distance(coordinates[u], coordinates[v])

        if u in target_graph.nodes and v in target_graph.nodes:
            target_graph.add_edge(u, v, weight=distance)  

        if u in full_graph.nodes and v in full_graph.nodes:
            full_graph.add_edge(u, v, weight=distance)

        if u in robot_graph.nodes and v in robot_graph.nodes:
            robot_graph.add_edge(u, v, weight=distance)
     
    return target_graph, full_graph, robot_graph

class EnvironmentManager(Node):
    """
    ROS2 Node to receive environment origin for a given env_id.
    
    Args:
        env_id (int): Environment identifier.
    """
    def __init__(self, env_id):
        super().__init__(f'env_{env_id}_origin_subscriber')
        self.env_id = env_id
        self.origin = None
        self.subscription = self.create_subscription(Point, f'/env_{env_id}/origin', self.origin_callback, 10)
        self.received_event = threading.Event()

    def origin_callback(self, msg):
        """
        Callback function to receive the origin.
        
        Args:
            msg (Point): The received ROS message containing x, y coordinates.
        """
        self.origin = [msg.x, msg.y]
        self.received_event.set()

def create_tree(env_id, tree_string, origin_offset, verbose = False):
    """
    Create the behavior tree after obtaining the correct environment origin.
    
    Args:
        env_id (int): Environment identifier.
        node (EnvironmentSubscriber): ROS2 node that subscribes to the environment origin.
    
    Returns:
        py_trees.composites.Selector: Root node of the behavior tree.
    """
    # Use received origin
    target_graph, full_graph, robot_graph = graph_init(origin_offset)
 
    # Define behavior nodes 
    patrol_node = PatrolNode(name = "PatrolNode", full_graph = full_graph, robot_graph = robot_graph, env_id = env_id, verbose=verbose)
    find_target_node = FindTargetNode(name='FindTarget', target_graph=target_graph, robot_graph=robot_graph, env_id=env_id, verbose=verbose)
    go_to_nearest_target = GoToNearestTarget(name="GoToNearestTarget",robot_graph=robot_graph, env_id=env_id, verbose=verbose)
    go_to_charger_node =GoToChargerNode(name="GoToCharger", robot_graph=robot_graph, env_id=env_id, verbose=verbose)
    go_to_spawn_node = GoToSpawnNode(name='GoToSpawnNode', robot_graph=robot_graph, env_id=env_id, verbose=verbose)
    picking_object_node = PickObject(name = 'PickObject', robot_graph=robot_graph, env_id=env_id, verbose=verbose)
    drop_object_node = DropObject(name='DropObject', env_id=env_id, verbose=verbose)
    charge_node = Charge(name='Charge', robot_graph = robot_graph, env_id=env_id, verbose=verbose)
    
    # define condition nodes
    is_robot_at_the_charger_node = IsRobotAtTheCharger(name='IsRobotAtTheCharger', env_id=env_id, robot_graph=robot_graph, verbose=verbose)
    is_robot_at_the_spawn_node = IsRobotAtTheSpawn(name='IsRobotAtTheSpawn', env_id=env_id, robot_graph=robot_graph, verbose=verbose)
    is_battery_on_proper_level = IsBatteryOnProperLevel(name='IsBatteryOnProperLevel', env_id=env_id, verbose=verbose)
    are_object_existed_on_internal_map = AreObjectsExistOnInternalMap(name='AreObjectExistsOnInternalMap', env_id=env_id, verbose=verbose)
    are_object_nearby_node = AreObjectNearby('AreObjectNearby', env_id=env_id, verbose=verbose)
    is_object_in_hand_node = IsObjectInHand('IsObjectInHand', env_id=env_id, verbose=verbose)
    is_nearby_object_not_at_goal = IsNearbyObjectNotAtGoal('IsNearbyObjectNotAtGoal', robot_graph=robot_graph, goal_node=[0], env_id=env_id, verbose=verbose)
    are_five_objects_at_spawn = AreFiveObjectsAtSpawn('AreFiveObjectsAtSpawn', robot_graph=robot_graph, env_id=env_id, verbose=verbose)

                    # Behaviors
    behavior_dict = {'a': patrol_node,
                     'b': find_target_node,
                     'c': go_to_nearest_target,
                     'd': go_to_charger_node,
                     'e': go_to_spawn_node,
                     'f': picking_object_node,
                     'g': drop_object_node,
                     'h': charge_node,
                    # Conditions
                     'A': is_robot_at_the_charger_node,
                     'B': is_robot_at_the_spawn_node,
                     'C': is_battery_on_proper_level,
                     'D': are_object_existed_on_internal_map,
                     'E': are_object_nearby_node,
                     'F': is_object_in_hand_node,
                     'G': is_nearby_object_not_at_goal,
                     'H': are_five_objects_at_spawn,
                     }
    
    cond_num = 0

    def string2tree(tree_string, cond_num):
        # Select Condition Node as Parent Node
        condition_node = tree_string[1]
        if condition_node == '0':
            parent = py_trees.composites.Sequence(f"Sequence_{cond_num}", memory=False)
        elif condition_node == '1':
            parent = py_trees.composites.Selector(f"Selector_{cond_num}", memory=False)
        elif condition_node == '2':
            parent = py_trees.composites.Parallel(f"Parallel_{cond_num}", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    
        cond_num += 1
        record = False

        for n in tree_string[2:]:
            if record:
                subtree_string += n
                if n == '(':
                    n_open += 1
                elif n == ')':
                    n_open -= 1

                if n_open == 0:
                    record = False
                    parent.add_child(string2tree(subtree_string, cond_num))
            else:
                if n == '(':
                    subtree_string = '('
                    n_open = 1
                    record = True
                elif n == ')':
                    pass
                else:
                    if n in behavior_dict.keys():
                        parent.add_child(behavior_dict[n])
                    else:
                        print('[Error] Undefined char in tree_string')

        return parent
    
    return string2tree(tree_string, cond_num)

def main(args=None):
    """
    Main function to initialize ROS2, create the behavior tree, and execute it.
    
    Args:
        args (list, optional): Command-line arguments passed to ROS2.
    """
    rclpy.init(args=args)

    env_id = args_cli.env_id
    tree_string = args_cli.bt_string
    
    env_node = EnvironmentManager(env_id)
    executor_thread = threading.Thread(target=rclpy.spin_once, args=(env_node,))
    executor_thread.start()

    env_node.received_event.wait()
    origin_offset = env_node.origin if env_node.origin else [0, 0]
    root = create_tree(env_id, tree_string, origin_offset, verbose=args_cli.verbose)

    node = rclpy.create_node(node_name=f"env_{env_id}_tree")
    tree = BehaviourTree(root=root)
    # Setup behavior tree
    try:
        tree.setup(node=node, timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    
    # Set parameters so tree can publish /env_{env_id}_tree/snapshots ros2 topic
    node.set_parameters([
        rclpy.parameter.Parameter("default_snapshot_stream", rclpy.Parameter.Type.BOOL, True)
    ])
    
    tree.tick_tock(period_ms=1000.0)
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Clean up
        subprocess.Popen(["ros2", "topic", "pub", f"/env_{env_id}/robot/cmd_vel", "geometry_msgs/msg/Twist", 
                          '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, '
                          '"angular": {"x": 0.0, "y": 0.0, "z": 0.0} }',"--once"])
        
        tree.shutdown()
        env_node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()