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
from rclpy.executors import ExternalShutdownException
from rclpy.context import Context

parser = argparse.ArgumentParser(description="Run your script with arguments.")
parser.add_argument('--env_id', type=int, help="Environment ID of the behavior tree.")
parser.add_argument('--bt_string', type=str, help="Behavior tree in string format.")
parser.add_argument('--number_of_target_to_success', type=int, default=5, help="Behavior tree in string format.")
parser.add_argument('--verbose', type=bool, default=False, help="Display behavior tree execution status in terminal.")

args_cli = parser.parse_args()

# Import behavior nodes
cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT/scripts/bt')
from behavior import (
    PatrolNode, MoveNode, GoToChargerNode, FindTargetNode, 
    AreObjectsExistOnInternalMap, GoToNearestTarget, AreObjectNearby, 
    PickObject, Charge, IsRobotAtTheCharger, IsRobotAtTheSpawn, 
    IsBatteryOnProperLevel, IsObjectInHand, DropObject, GoToSpawnNode, 
    IsNearbyObjectNotAtGoal, AreXObjectsAtSpawn
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
    def __init__(self, env_id, context=None):
        super().__init__(f'env_{env_id}_origin_subscriber', context=context)
        self.env_id = env_id
        self.origin = None
        self.subscription = self.create_subscription(Point, f'/env_{env_id}/origin', self.origin_callback, 10)
        self.cmd_vel_publishers = self.create_publisher(Twist, f"/env_{env_id}/robot/cmd_vel", 10)
        self.received_event = threading.Event()

    def origin_callback(self, msg):
        """
        Callback function to receive the origin.
        
        Args:
            msg (Point): The received ROS message containing x, y coordinates.
        """
        self.origin = [msg.x, msg.y]
        self.received_event.set()

    def publish_zero_cmd_vel(self):
        self.cmd_vel_publishers.publish(Twist())

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

                    # Behaviors
    behavior_dict = {'a': lambda ind: PatrolNode(name = f"PatrolNode_{ind}", full_graph = full_graph, robot_graph = robot_graph, env_id = env_id, verbose=verbose),
                     'b': lambda ind: FindTargetNode(name=f'FindTarget_{ind}', target_graph=target_graph, robot_graph=robot_graph, env_id=env_id, verbose=verbose),
                     'c': lambda ind: GoToNearestTarget(name=f"GoToNearestTarget_{ind}",robot_graph=robot_graph, env_id=env_id, verbose=verbose),
                     'd': lambda ind: GoToChargerNode(name=f"GoToCharger_{ind}", robot_graph=robot_graph, env_id=env_id, verbose=verbose),
                     'e': lambda ind: GoToSpawnNode(name=f'GoToSpawnNode_{ind}', robot_graph=robot_graph, env_id=env_id, verbose=verbose),
                     'f': lambda ind: PickObject(name = f'PickObject_{ind}', robot_graph=robot_graph, env_id=env_id, verbose=verbose),
                     'g': lambda ind: DropObject(name=f'DropObject_{ind}', env_id=env_id, verbose=verbose),
                     'h': lambda ind: Charge(name=f'Charge_{ind}', robot_graph = robot_graph, env_id=env_id, verbose=verbose),
                    # Conditions
                     'A': lambda ind: IsRobotAtTheCharger(name=f'IsRobotAtTheCharger_{ind}', env_id=env_id, robot_graph=robot_graph, verbose=verbose),
                     'B': lambda ind: IsRobotAtTheSpawn(name=f'IsRobotAtTheSpawn_{ind}', env_id=env_id, robot_graph=robot_graph, verbose=verbose),
                     'C': lambda ind: IsBatteryOnProperLevel(name=f'IsBatteryOnProperLevel_{ind}', env_id=env_id, verbose=verbose),
                     'D': lambda ind: AreObjectsExistOnInternalMap(name=f'AreObjectExistsOnInternalMap_{ind}', env_id=env_id, verbose=verbose),
                     'E': lambda ind: AreObjectNearby(f'AreObjectNearby_{ind}', env_id=env_id, verbose=verbose),
                     'F': lambda ind: IsObjectInHand(f'IsObjectInHand_{ind}', env_id=env_id, verbose=verbose),
                     'G': lambda ind: IsNearbyObjectNotAtGoal(f'IsNearbyObjectNotAtGoal_{ind}', robot_graph=robot_graph, goal_node=[0], env_id=env_id, verbose=verbose),
                     'H': lambda ind: AreXObjectsAtSpawn(f'AreFiveObjectsAtSpawn_{ind}', robot_graph=robot_graph, env_id=env_id, target_number=args_cli.number_of_target_to_success,verbose=verbose),
                     }

    def string2tree(tree_string, cond_num):
        # If a single behavior node is passed, return the corresponding behavior node
        if len(tree_string) == 1:
            return behavior_dict[tree_string[0]](0)
        
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

        child_num = 0
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
                        parent.add_child(behavior_dict[n](child_num))
                        child_num += 1
                    else:
                        print('[Error] Undefined char in tree_string')

        return parent
    
    if tree_string=='':
        return None
    
    return string2tree(tree_string, 0)

def main():
    env_id = args_cli.env_id
    tree_string = args_cli.bt_string

    # Create a custom context
    my_context = Context()
    rclpy.init(context=my_context)

    env_node = EnvironmentManager(env_id, context=my_context)
    stop_event = threading.Event()

    def spin_env_node(node, stop_event):
        try:
            executor = rclpy.executors.SingleThreadedExecutor(context=my_context)
            executor.add_node(node)
            while rclpy.ok(context=my_context) and not stop_event.is_set():
                executor.spin_once(timeout_sec=0.1)
        except Exception as e:
            if args_cli.verbose: print(f"[WARN] Spin thread exception: {e}")
        finally:
            if args_cli.verbose: print(f"[INFO] Spin thread for {node.get_name()} exited cleanly.")

    executor_thread = threading.Thread(target=spin_env_node, args=(env_node, stop_event))
    executor_thread.start()

    # ==== WAIT FOR ORIGIN ====
    env_node.received_event.wait()
    origin_offset = env_node.origin if env_node.origin else [0, 0]

    # ==== NOW STOP ENV NODE ====
    stop_event.set()

    # ==== Build and Spin the BT ====
    root = create_tree(env_id, tree_string, origin_offset, verbose=args_cli.verbose)

    if root is not None:
        node = rclpy.create_node(node_name=f"env_{env_id}_tree", context=my_context)
        tree = BehaviourTree(root=root)
        try:
            tree.setup(node=node, timeout=15)
        except py_trees_ros.exceptions.TimedOutError as e:
            console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
            tree.shutdown()
            node.destroy_node()
            return
        except KeyboardInterrupt:
            console.logerror("tree setup interrupted")
            tree.shutdown()
            node.destroy_node()
            return

        node.set_parameters([
            rclpy.parameter.Parameter("default_snapshot_stream", rclpy.Parameter.Type.BOOL, True)
        ])

        if args_cli.verbose: print(f"[INFO] running tree {args_cli.env_id}")
        tree.tick_tock(period_ms=1000.0)

        try:
            executor = rclpy.executors.SingleThreadedExecutor(context=my_context)
            executor.add_node(tree.node)
            executor.spin()
        except (ExternalShutdownException, KeyboardInterrupt):
            if args_cli.verbose: print("[INFO] Shutdown requested (External or Ctrl+C).")
        finally:
            if args_cli.verbose: print("[INFO] Cleaning up...")
            tree.shutdown()
            node.destroy_node()

            executor_thread.join()
            env_node.destroy_node()

if __name__ == '__main__':
    main()
