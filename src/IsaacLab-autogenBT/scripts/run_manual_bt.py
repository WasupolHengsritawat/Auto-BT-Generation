import rclpy
import py_trees
import py_trees_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import random
import py_trees.console as console
import sys

import math
import networkx as nx

from py_trees_ros.trees import BehaviourTree
from behavior import PatrolNode, MoveNode, GoToChargerNode, FindTargetNode

def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def graph_init():
    # Coordinates extracted manually from the cave map
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
                   ( 14.21, -3.47), # 20 - target  #1
                   ( 21.61, -0.59), # 21 - target  #2
                   ( 19.18,  7.24), # 22 - target  #3
                   ( 20.16, 13.45), # 23 - target  #4
                   ( 22.52, 20.71), # 24 - target  #5
                   (  9.40, 24.10), # 25 - target  #6
                   ( -1.55, 22.86), # 26 - target  #7
                   ( -0.94, 13.28), # 27 - target  #8
                   ( -2.14,  7.38), # 28 - target  #9
                   (  8.34,  7.16)] # 29 - target #10
    
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
    full_graph = nx.Graph()
    robot_graph = nx.Graph()
    target_graph = nx.Graph()

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

def create_tree():

    target_graph, full_graph, robot_graph = graph_init()

    root = py_trees.composites.Parallel("Test Behavior", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    patrol_node = PatrolNode(name = "Patrol Node", full_graph = full_graph, robot_graph = robot_graph, env_id = 0)
    root.add_child(patrol_node)
    findtarget_node = FindTargetNode(name='FindTarget', target_graph=target_graph, robot_graph=robot_graph, env_id=0)
    root.add_child(findtarget_node)
    # gotocharger_node = GoToChargerNode(name='GoToChargerNode', robot_graph = robot_graph, env_id=0)
    # root.add_child(gotocharger_node)

    return root

def main(args=None):
    rclpy.init(args=args)
    root = create_tree()
    tree = BehaviourTree(root)
    
    # Behavior Tree Setup
    try:
        tree.setup(timeout=15)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    tree.tick_tock(period_ms=1000.0)

    # Behavior Tree Execution
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

main()