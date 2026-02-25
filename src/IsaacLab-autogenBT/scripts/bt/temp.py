import os
import sys
import math
import threading
import signal
import time
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

from py_trees.common import Status

# Import local files
# Get the absolute path to the directory containing this script and the root of the project
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
simulation_dir = os.path.abspath(os.path.join(project_root, "scripts", "simulation"))
bt_dir = os.path.abspath(os.path.join(project_root, "scripts", "bt"))

# Add it to sys.path 
sys.path.insert(0, project_root)
sys.path.insert(0, script_dir)
sys.path.insert(0, simulation_dir)
sys.path.insert(0, bt_dir)

from ros2_nodes.ros2_bt_tracker import BTStatusTrackerNode
from simple_bt_manager import run_simple_BTs, stop_simple_BTs


# bt_string_array = ["(1(0(0B(3D)(3E)(3F)(3H))(2ab))(0(0(3B)D(3E)(3F)(3H))c)(0(0(3B)DE(3F)(3H))f)(0(0(3B)(3D)(3E)F(3H))e)(0(0B(3D)(3E)F(3H))(1gf)))"]
# bt_string_array = ["(1(0(0B(3D)(3E)(3F)(3H))(2ba))(0(0(3B)D(3E)(3F)(3H))c)(0(0(3B)DE(3F)(3H))f)(0(0(3B)(3D)(3E)F(3H))e)(0(0B(3D)(3E)F(3H))g))"]
bt_string_array = ["(1(0(0B(3D)(3E)(3F)(3H))(2a))(0(0(3B)(3D)(3E)(3F)(3H))(2ba))(0(0(3B)D(3E)(3F)(3H))c)(0(0(3B)DE(3F)(3H))f)(0(0(3B)(3D)(3E)F(3H))e)(0(0B(3D)(3E)F(3H))g))"]

rclpy.init()

bt_tracker = BTStatusTrackerNode(num_envs=1)
run_simple_BTs(bt_string_array)

while  not (bt_tracker.get_status(env_id=0) in ['SUCCESS', 'FAILURE']):
    # Spin BT Tracker Node
    try:
        rclpy.spin_once(bt_tracker, timeout_sec=0.0)
    except rclpy.executors.ExternalShutdownException:
        print("[INFO] Spin exited because ROS2 shutdown detected.")

stop_simple_BTs()

print("Final status:", bt_tracker.get_status(env_id=0))

# ===================================================================================================

# parser = argparse.ArgumentParser(description="Run your script with arguments.")
# parser.add_argument('--env_id', type=int, help="Environment ID of the behavior tree.")
# parser.add_argument('--bt_string', type=str, help="Behavior tree in string format.")
# parser.add_argument('--verbose', type=bool, default=False, help="Display behavior tree execution status in terminal.")

# args_cli = parser.parse_args()

# # Import behavior nodes
# cwd = os.getcwd()
# sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT/scripts/bt')
# from simple_ros2_behavior import (
#     PatrolNode, FindTargetNode, AreObjectsExistOnInternalMap, 
#     GoToNearestTarget, AreObjectNearby, PickObject, IsRobotAtTheSpawn, 
#     IsObjectInHand, DropObject, GoToSpawnNode, AreXObjectsAtSpawn
# )

# def create_tree(tree_string, env_id = 0, verbose = False):
#     """
#     Create the behavior tree after obtaining the correct environment origin.
    
#     Args:
#         env_id (int): Environment identifier.
#         node (EnvironmentSubscriber): ROS2 node that subscribes to the environment origin.
    
#     Returns:
#         py_trees.composites.Selector: Root node of the behavior tree.
#     """

#                     # Behaviors
#     behavior_dict = {'a': lambda ind: PatrolNode(name = f"PatrolNode_{ind}", env_id = env_id, verbose=verbose),
#                     'b': lambda ind: FindTargetNode(name=f'FindTarget_{ind}', env_id=env_id, verbose=verbose),
#                     'c': lambda ind: GoToNearestTarget(name=f"GoToNearestTarget_{ind}", env_id=env_id, verbose=verbose),
#                     'e': lambda ind: GoToSpawnNode(name=f'GoToSpawnNode_{ind}', env_id=env_id, verbose=verbose),
#                     'f': lambda ind: PickObject(name = f'PickObject_{ind}', env_id=env_id, verbose=verbose),
#                     'g': lambda ind: DropObject(name=f'DropObject_{ind}', env_id=env_id, verbose=verbose),
#                     # Conditions
#                     'B': lambda ind: IsRobotAtTheSpawn(name=f'IsRobotAtTheSpawn_{ind}', env_id=env_id, verbose=verbose),
#                     'D': lambda ind: AreObjectsExistOnInternalMap(name=f'AreObjectExistsOnInternalMap_{ind}', env_id=env_id, verbose=verbose),
#                     'E': lambda ind: AreObjectNearby(f'AreObjectNearby_{ind}', env_id=env_id, verbose=verbose),
#                     'F': lambda ind: IsObjectInHand(f'IsObjectInHand_{ind}', env_id=env_id, verbose=verbose),
#                     'H': lambda ind: AreXObjectsAtSpawn(f'AreFiveObjectsAtSpawn_{ind}', env_id=env_id, verbose=verbose),
#                     }

#     def string2tree(tree_string, cond_num):
#         # If a single behavior node is passed, return the corresponding behavior node
#         if len(tree_string) == 1:
#             return behavior_dict[tree_string[0]](0)
        
#         is_decorator = False
        
#         # Select Condition Node as Parent Node
#         condition_node = tree_string[1]
#         if condition_node == '0':
#             parent = py_trees.composites.Sequence(f"Sequence_{cond_num}", memory=False)
#         elif condition_node == '1':
#             parent = py_trees.composites.Selector(f"Selector_{cond_num}", memory=False)
#         elif condition_node == '2':
#             parent = py_trees.composites.Parallel(f"Parallel_{cond_num}", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
#         elif condition_node == '3':
#             parent = py_trees.decorators.Inverter(f'Inverter_{cond_num}', behavior_dict[tree_string[2]](0))
#             is_decorator = True
    
#         cond_num += 1
#         record = False

#         child_num = 0
#         for n in tree_string[2:]:
            
#             if is_decorator:
#                 break
    
#             if record:
#                 subtree_string += n
#                 if n == '(':
#                     n_open += 1
#                 elif n == ')':
#                     n_open -= 1

#                 if n_open == 0:
#                     record = False
#                     parent.add_child(string2tree(subtree_string, cond_num))
#             else:
#                 if n == '(':
#                     subtree_string = '('
#                     n_open = 1
#                     record = True
#                 elif n == ')':
#                     pass
#                 else:
#                     if n in behavior_dict.keys():
#                         parent.add_child(behavior_dict[n](child_num))
#                         child_num += 1
#                     else:
#                         print('[Error] Undefined char in tree_string')

#         return parent
    
#     if tree_string=='':
#         return None
    
#     return string2tree(tree_string, 0)

# def main():
#     env_id = 0
#     tree_string = "B"
#     # print(tree_string)

#     # Create a custom context
#     my_context = Context()
#     rclpy.init(context=my_context)

#     # ==== Build and Spin the BT ====
#     root = create_tree(tree_string, env_id, verbose=args_cli.verbose)

#     if root is not None:
#         # [1] Setup BT
#         node = rclpy.create_node(node_name=f"env_{env_id}_tree", context=my_context)
#         tree = BehaviourTree(root=root)
#         try:
#             tree.setup(node=node, timeout=15)
#         except py_trees_ros.exceptions.TimedOutError as e:
#             console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
#             tree.shutdown()
#             node.destroy_node()
#             return
#         except KeyboardInterrupt:
#             console.logerror("tree setup interrupted")
#             tree.shutdown()
#             node.destroy_node()
#             return

#         node.set_parameters([
#             rclpy.parameter.Parameter("default_snapshot_stream", rclpy.Parameter.Type.BOOL, True)
#         ])

#         # [2] Define pretick handler
#         def get_bt_status(tree):
#             # status = tree.root.status
#             print(f"[INFO] BT Status: {tree.root.status}")

#             if tree.root.status in [py_trees.common.Status.SUCCESS, py_trees.common.Status.FAILURE]:
#                 executor.shutdown()

#         # [3] Execute the BT
#         try:
#             if args_cli.verbose: print(f"[INFO] running tree {args_cli.env_id}")
#             tree.tick_tock(period_ms=10.0, number_of_iterations=5, pre_tick_handler=get_bt_status)

#             executor = rclpy.executors.SingleThreadedExecutor(context=my_context)
#             executor.add_node(tree.node)
#             executor.spin()
#         except (ExternalShutdownException, KeyboardInterrupt, SystemExit):
#             if args_cli.verbose: print("[INFO] Shutdown requested (External or Ctrl+C).")
#         finally:
#             if args_cli.verbose: print("[INFO] Cleaning up...")
#             tree.shutdown()
#             node.destroy_node()

# if __name__ == '__main__':
#     main()
