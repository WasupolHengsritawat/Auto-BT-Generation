import os
import sys
import math
import threading
import signal
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
parser.add_argument('--verbose', type=bool, default=False, help="Display behavior tree execution status in terminal.")

args_cli = parser.parse_args()

# Import behavior nodes
cwd = os.getcwd()
sys.path.insert(0, f'{cwd}/src/IsaacLab-autogenBT/scripts/bt')
from simple_ros2_behavior import (
    PatrolNode, FindTargetNode, AreObjectsExistOnInternalMap, 
    GoToNearestTarget, AreObjectNearby, PickObject, IsRobotAtTheSpawn, 
    IsObjectInHand, DropObject, GoToSpawnNode, AreXObjectsAtSpawn
)

def create_tree(tree_string, env_id = 0, verbose = False):
    """
    Create the behavior tree after obtaining the correct environment origin.
    
    Args:
        env_id (int): Environment identifier.
        node (EnvironmentSubscriber): ROS2 node that subscribes to the environment origin.
    
    Returns:
        py_trees.composites.Selector: Root node of the behavior tree.
    """

                    # Behaviors
    behavior_dict = {'a': lambda ind: PatrolNode(name = f"PatrolNode_{ind}", env_id = env_id, verbose=verbose),
                    'b': lambda ind: FindTargetNode(name=f'FindTarget_{ind}', env_id=env_id, verbose=verbose),
                    'c': lambda ind: GoToNearestTarget(name=f"GoToNearestTarget_{ind}", env_id=env_id, verbose=verbose),
                    'e': lambda ind: GoToSpawnNode(name=f'GoToSpawnNode_{ind}', env_id=env_id, verbose=verbose),
                    'f': lambda ind: PickObject(name = f'PickObject_{ind}', env_id=env_id, verbose=verbose),
                    'g': lambda ind: DropObject(name=f'DropObject_{ind}', env_id=env_id, verbose=verbose),
                    # Conditions
                    'B': lambda ind: IsRobotAtTheSpawn(name=f'IsRobotAtTheSpawn_{ind}', env_id=env_id, verbose=verbose),
                    'D': lambda ind: AreObjectsExistOnInternalMap(name=f'AreObjectExistsOnInternalMap_{ind}', env_id=env_id, verbose=verbose),
                    'E': lambda ind: AreObjectNearby(f'AreObjectNearby_{ind}', env_id=env_id, verbose=verbose),
                    'F': lambda ind: IsObjectInHand(f'IsObjectInHand_{ind}', env_id=env_id, verbose=verbose),
                    'H': lambda ind: AreXObjectsAtSpawn(f'AreFiveObjectsAtSpawn_{ind}', env_id=env_id, verbose=verbose),
                    }

    def string2tree(tree_string, cond_num):
        # If a single behavior node is passed, return the corresponding behavior node
        if len(tree_string) == 1:
            return behavior_dict[tree_string[0]](0)
        
        is_decorator = False
        
        # Select Condition Node as Parent Node
        condition_node = tree_string[1]
        if condition_node == '0':
            parent = py_trees.composites.Sequence(f"Sequence_{cond_num}", memory=False)
        elif condition_node == '1':
            parent = py_trees.composites.Selector(f"Selector_{cond_num}", memory=False)
        elif condition_node == '2':
            parent = py_trees.composites.Parallel(f"Parallel_{cond_num}", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
        elif condition_node == '3':
            parent = py_trees.decorators.Inverter(f'Inverter_{cond_num}', behavior_dict[tree_string[2]](0))
            is_decorator = True
    
        cond_num += 1
        record = False

        child_num = 0
        for n in tree_string[2:]:
            
            if is_decorator:
                break
    
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
    # print(tree_string)

    # Create a custom context
    my_context = Context()
    rclpy.init(context=my_context)

    # ==== Build and Spin the BT ====
    root = create_tree(tree_string, env_id, verbose=args_cli.verbose)

    if root is not None:
        # [1] Setup BT
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

        # [2] Handle SIGTERM for graceful shutdown
        def handle_sigterm(signum, frame):
            if args_cli.verbose: print("[INFO] Cleaning up...")
            tree.shutdown()
            node.destroy_node()

        signal.signal(signal.SIGTERM, handle_sigterm)

        # [3] Execute the BT
        try:
            if args_cli.verbose: print(f"[INFO] running tree {args_cli.env_id}")
            
            executor = rclpy.executors.SingleThreadedExecutor(context=my_context)
            executor.add_node(tree.node)

            # Start the ticking
            tree.tick_tock(period_ms=10.0)

            # Use a more robust loop
            while rclpy.ok(context=my_context):
                # Wrap spin_once in a try-except to catch the "Destroyable" error mid-spin
                try:
                    executor.spin_once(timeout_sec=0.05)
                except rclpy._rclpy_pybind11.InvalidHandle:
                    # If we hit this, the node/entities are already dying; just break
                    break

        except (ExternalShutdownException, KeyboardInterrupt, SystemExit):
            if args_cli.verbose: print("[INFO] Shutdown requested.")
        finally:
            # --- CRITICAL CLEANUP ORDER ---
            if args_cli.verbose: print("[INFO] Cleaning up...")
            
            # 1. Stop the tick_tock background thread immediately
            # Use interrupt() instead of shutdown() first to stop the loop
            tree.interrupt()
            
            # 2. Remove node from executor so no more callbacks are processed
            executor.remove_node(tree.node)
            
            # 3. Shutdown the tree (be defensive)
            try:
                tree.shutdown()
            except Exception as e:
                if args_cli.verbose: print(f"[DEBUG] Tree shutdown suppressed: {e}")

            # 4. Destroy the node and shutdown ROS context
            if rclpy.ok(context=my_context):
                node.destroy_node()
                # Only shutdown if we started it
                rclpy.shutdown(context=my_context)

if __name__ == '__main__':
    main()
