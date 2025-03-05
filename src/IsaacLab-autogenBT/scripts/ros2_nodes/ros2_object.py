import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import numpy as np
import torch
from omni.isaac.lab.scene import InteractiveScene
from threading import Thread

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
        if self.host != None:
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

class ObjectGroupNode(Node):
    def __init__(self, name, pos_list: np.ndarray | torch.Tensor, env_id: int, scene: InteractiveScene):
        super().__init__(f'{name}_obj_group_node')
        
        self.name = name
        self.env_id = env_id
        self.scene = scene
        self.num_objs = pos_list.shape[0]
        self.objs = []
        self.target_transform_pub = []

        # Initialize ROS messages
        qos_profile = QoSProfile(depth=10)

        for i in range(self.num_objs): 
            # Initialize a list of objects  
            self.objs.append(Object(pos_list[i], self.env_id, self.scene))

            # ROS2 publisher
            self.target_transform_pub.append(self.create_publisher(PointStamped, f'env_{self.env_id}/target{i}/tf', qos_profile))
            
        # ROS2 service server
        self.create_service(PickingRequest, f"env_{self.env_id}/picking_req", self.pick_callback)

        # ROS2 Update timer
        self.update_timer = self.create_timer(0.05, self.update_objs_pos)

        self.get_logger().info(f"ObjectGroupNode node {name} initialized.")
            
    def pick_callback(self, req, res):
        host = req.host_name.data
        obj_id = req.obj_id.data
        status = req.status.data

        if status:
            self.objs[obj_id].pick(host)
        else:
            self.objs[obj_id].drop()

        return PickingRequest.Response()

    def update_objs_pos(self):
        # Publish objects position
        for i in range(self.num_objs):  
            # Header
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"

            # Data
            obj_pos = self.objs[i].get_pos()
            msg.point.x = float(obj_pos[0])
            msg.point.y = float(obj_pos[1])
            msg.point.z = float(obj_pos[2])

            self.target_transform_pub[i].publish(msg)
    
    def repos(self, pos_list: np.ndarray | torch.Tensor):
        for i in range(self.num_objs):
            self.objs[i].repos(pos_list[i])

class ObjectGroupManager:
    def __init__(self, obj_group_name, pos_llist: np.ndarray | torch.Tensor, scene: InteractiveScene):
        self.obj_groups = [ObjectGroupNode(name, pos_list, env_id, scene) for env_id, (name, pos_list) in enumerate(zip(obj_group_name,pos_llist))]

        self.executor_thread = Thread(target=self.run_executor, daemon=True)
        self.executor_thread.start()

    def repos(self, pos_llist: np.ndarray):
        for env_id, pos_list in enumerate(pos_llist):
            self.obj_groups[env_id].repos(pos_list)

    def run_executor(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        for obj_group in self.obj_groups:
            executor.add_node(obj_group)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for obj_group in self.obj_groups:
                obj_group.destroy_node()

    def stop(self):
        # Ensure the nodes shut down gracefully
        self.executor_thread.join()