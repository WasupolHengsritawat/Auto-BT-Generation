import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import numpy as np
import torch
from omni.isaac.lab.scene import InteractiveScene
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

from autogen_bt_interface.srv import PickingRequest

class Object():
    def __init__(self, pos: list | tuple | np.ndarray | torch.Tensor, env_id: int, scene: InteractiveScene):
        self.env_id = env_id
        self.scene = scene
        self.pos = pos
        self.status = 'settled'
        self.host = None

    def pick(self, host: str = "robot"):
        self.status = 'picked'
        self.host = host

    def drop(self):
        if self.host is not None:
            self.pos = self.scene[self.host].data.root_state_w[self.env_id, :3]
            self.status = 'settled'
            self.host = None

    def get_pos(self):
        if self.status == 'settled':
            return self.pos
        elif self.status == 'picked':
            return self.scene[self.host].data.root_state_w[self.env_id, :3]
        else:
            raise ValueError('Invalid object status.')

    def repos(self, pos: list | tuple | np.ndarray | torch.Tensor):
        self.pos = pos
        self.status = 'settled'
        self.host = None

class ObjectGroupManagerNode(Node):
    def __init__(self, obj_group_name, pos_llist: np.ndarray | torch.Tensor, scene: InteractiveScene):
        """
        A single ROS2 node to manage multiple object groups across environments.

        :param obj_group_name: List of names for each object group/environment.
        :param pos_llist: List of position arrays per environment.
        :param scene: Reference to the IsaacLab interactive scene.
        """
        super().__init__('object_group_manager_node')

        self.scene = scene
        self.objs_by_env = {}        # {env_id: [Object, Object, ...]}
        self.target_transform_pub = {}  # {env_id: [Publisher, Publisher, ...]}
        self.num_envs = len(obj_group_name)

        qos_profile = QoSProfile(depth=10)

        for env_id, (name, pos_list) in enumerate(zip(obj_group_name, pos_llist)):
            self.objs_by_env[env_id] = []
            self.target_transform_pub[env_id] = []

            for i in range(pos_list.shape[0]):
                # Initialize each object
                obj = Object(pos_list[i], env_id, self.scene)
                self.objs_by_env[env_id].append(obj)

                # Create a publisher for the object's pose
                pub = self.create_publisher(PointStamped, f'env_{env_id}/target{i}/tf', qos_profile)
                self.target_transform_pub[env_id].append(pub)

            # Create a picking service for this environment
            self.create_service(PickingRequest, f"env_{env_id}/picking_req", self.make_pick_callback(env_id))

            self.get_logger().info(f"Object group for env {env_id} initialized.")

        # ROS2 update timer (20 Hz)
        self.update_timer = self.create_timer(0.05, self.update_objs_pos)

    def make_pick_callback(self, env_id):
        """
        Factory method to create a unique picking callback for each environment.

        :param env_id: The ID of the environment.
        :return: Callback function for service server.
        """
        def callback(req, res):
            host = req.host_name.data
            obj_id = req.obj_id.data
            status = req.status.data

            if obj_id < len(self.objs_by_env[env_id]):
                if status:
                    self.objs_by_env[env_id][obj_id].pick(host)
                else:
                    self.objs_by_env[env_id][obj_id].drop()

            return PickingRequest.Response()
        return callback

    def update_objs_pos(self):
        """
        Publishes the latest position of all objects in all environments.
        """
        for env_id, obj_list in self.objs_by_env.items():
            for i, obj in enumerate(obj_list):
                msg = PointStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "world"

                pos = obj.get_pos()
                msg.point.x = float(pos[0])
                msg.point.y = float(pos[1])
                msg.point.z = float(pos[2])

                self.target_transform_pub[env_id][i].publish(msg)

    def repos(self, pos_llist: np.ndarray | torch.Tensor):
        """
        Reset object positions in each environment.

        :param pos_llist: List of new position arrays.
        """
        for env_id, pos_list in enumerate(pos_llist):
            for i, pos in enumerate(pos_list):
                self.objs_by_env[env_id][i].repos(pos)

class ObjectGroupManager:
    def __init__(self, obj_group_name, pos_llist: np.ndarray | torch.Tensor, scene: InteractiveScene):
        """
        Class to launch the ObjectGroupManagerNode in a background thread using MultiThreadedExecutor.

        :param obj_group_name: List of group names.
        :param pos_llist: List of position arrays per environment.
        :param scene: IsaacLab scene object.
        """
        self.node = ObjectGroupManagerNode(obj_group_name, pos_llist, scene)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        # Start ROS2 executor in background thread
        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def repos(self, pos_llist: np.ndarray):
        """
        Reset all objects' positions.

        :param pos_llist: List of new position arrays per environment.
        """
        self.node.repos(pos_llist)

    def stop(self):
        """
        Stop the ROS2 node and executor thread cleanly.
        """
        self.executor.shutdown()
        self.node.destroy_node()
        self.executor_thread.join()