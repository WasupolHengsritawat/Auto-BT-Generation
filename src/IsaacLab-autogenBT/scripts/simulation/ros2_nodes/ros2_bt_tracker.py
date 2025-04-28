import rclpy
from rclpy.node import Node
from threading import Thread
from py_trees_ros_interfaces.msg import BehaviourTree as BTMsg
from unique_identifier_msgs.msg import UUID

from rclpy.executors import MultiThreadedExecutor

class BTStatusTrackerNode(Node):
    def __init__(self, num_envs: int):
        """
        A single ROS2 node to track Behavior Tree status for multiple environments.

        :param num_envs: Number of environments to track.
        """
        super().__init__('bt_status_tracker_node')
        self.num_envs = num_envs
        self.root_statuses = {}  # {env_id: status}

        for env_id in range(num_envs):
            topic = f"/env_{env_id}_tree/snapshots"
            self.root_statuses[env_id] = None

            self.create_subscription(
                BTMsg,
                topic,
                self.make_callback(env_id),
                10
            )

        self.get_logger().info(f"BTStatusTrackerNode tracking {num_envs} environments.")

    def make_callback(self, env_id):
        """
        Factory function to generate a unique BT snapshot callback for each environment.
        """
        def callback(msg: BTMsg):
            for behavior in msg.behaviours:
                if list(behavior.parent_id.uuid) == [0] * 16:   # root has no parent
                    self.root_statuses[env_id] = behavior.status
                    break
        return callback

    def get_status(self, env_id: int) -> str:
        """
        Get the root node status as a human-readable string for a specific environment.

        :param env_id: The environment ID.
        :return: Status string.
        """
        status_map = {
            0: 'FAILURE',
            1: 'INVALID',
            2: 'RUNNING',
            3: 'SUCCESS'
        }
        return status_map.get(self.root_statuses.get(env_id), 'UNKNOWN')
