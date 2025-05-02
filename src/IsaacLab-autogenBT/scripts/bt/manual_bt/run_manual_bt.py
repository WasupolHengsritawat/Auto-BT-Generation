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
from geometry_msgs.msg import Point

# Import behavior nodes
cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT/scripts')
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

class EnvironmentSubscriber(Node):
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

def create_tree(env_id, origin_offset, verbose = True):
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
    is_robot_at_the_charger_node = IsRobotAtTheCharger(name='IsRobotAtTheCharger', env_id=env_id, verbose=verbose)
    is_robot_at_the_spawn_node = IsRobotAtTheSpawn(name='IsRobotAtTheSpawn', env_id=env_id, verbose=verbose)
    is_battery_on_proper_level = IsBatteryOnProperLevel(name='IsBatteryOnProperLevel', env_id=env_id, verbose=verbose)
    are_object_existed_on_internal_map = AreObjectsExistOnInternalMap(name='AreObjectExistsOnInternalMap', env_id=env_id, verbose=verbose)
    are_object_nearby_node = AreObjectNearby('AreObjectNearby', env_id=env_id, verbose=verbose)
    is_object_in_hand_node = IsObjectInHand('IsObjectInHand', env_id=env_id, verbose=verbose)
    is_nearby_object_not_at_goal = IsNearbyObjectNotAtGoal('IsNearbyObjectNotAtGoal', robot_graph=robot_graph, goal_node=[0], env_id=env_id, verbose=verbose)
    are_five_objects_at_spawn = AreXObjectsAtSpawn('AreFiveObjectsAtSpawn', robot_graph=robot_graph, env_id=env_id, verbose=verbose)

    # Define tree structure
    parallel_1 = py_trees.composites.Parallel("Parallel_1", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    parallel_1.add_child(patrol_node)
    parallel_1.add_child(find_target_node)

    selector_1 = py_trees.composites.Selector("Selector_1", memory=False)
    selector_1.add_child(are_object_existed_on_internal_map)
    selector_1.add_child(parallel_1)

    sequence_1 = py_trees.composites.Sequence("Sequence_1", memory=False)
    sequence_1.add_child(selector_1)
    sequence_1.add_child(go_to_nearest_target)

    sequence_6 = py_trees.composites.Sequence("Sequence_6", memory=False)
    sequence_6.add_child(are_object_nearby_node)
    sequence_6.add_child(is_nearby_object_not_at_goal)

    selector_2 = py_trees.composites.Selector("Selector_2", memory=False)
    selector_2.add_child(sequence_6)
    selector_2.add_child(sequence_1)

    sequence_2 = py_trees.composites.Sequence("Sequence_2", memory=False)
    sequence_2.add_child(selector_2)
    sequence_2.add_child(picking_object_node)

    selector_3 = py_trees.composites.Selector("Selector_3", memory=False)
    selector_3.add_child(is_object_in_hand_node)
    selector_3.add_child(sequence_2)

    selector_4 = py_trees.composites.Selector("Selector_4", memory=False)
    selector_4.add_child(is_robot_at_the_charger_node)
    selector_4.add_child(go_to_charger_node)

    sequence_3 = py_trees.composites.Sequence("Sequence_4", memory=False)
    sequence_3.add_child(selector_4)
    sequence_3.add_child(charge_node)

    selector_5 = py_trees.composites.Selector("Selector_5", memory=False)
    selector_5.add_child(is_battery_on_proper_level)
    selector_5.add_child(sequence_3)

    selector_6 = py_trees.composites.Selector("Selector_6", memory=False)
    selector_6.add_child(is_robot_at_the_spawn_node)
    selector_6.add_child(go_to_spawn_node)

    sequence_4 = py_trees.composites.Sequence("Sequence_4", memory=False)
    sequence_4.add_child(selector_3)
    sequence_4.add_child(selector_6)
    sequence_4.add_child(drop_object_node)

    sequence_5 = py_trees.composites.Sequence("Sequence_5", memory=False)
    sequence_5.add_child(selector_5)
    sequence_5.add_child(sequence_4)

    selector_7 = py_trees.composites.Selector("Selector_7", memory=False)
    selector_7.add_child(are_five_objects_at_spawn)
    selector_7.add_child(sequence_5)

    return selector_7

def main(args=None):
    """
    Main function to initialize ROS2, create the behavior tree, and execute it.
    
    Args:
        args (list, optional): Command-line arguments passed to ROS2.
    """
    rclpy.init(args=args)
    env_id = 2
    
    node = EnvironmentSubscriber(env_id)
    executor_thread = threading.Thread(target=rclpy.spin_once, args=(node,))
    executor_thread.start()

    node.received_event.wait()
    origin_offset = node.origin if node.origin else [0, 0]
    root = create_tree(env_id, origin_offset)
    tree = BehaviourTree(root)
    
    # Setup behavior tree
    try:
        tree.setup(node_name=f"env_{env_id}_tree", timeout=15)
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
    
    tree.tick_tock(period_ms=1000.0)
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()