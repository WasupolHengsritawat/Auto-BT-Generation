import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from threading import Thread
import numpy as np
from typing import List
import torch

class RobotDriverNode(Node):
    def __init__(self, env_id: int, wheel_distance: float, wheel_radius: float, max_wheel_speed: float):
        super().__init__(f'env_{env_id}_driver_node')
        self.env_id = env_id
        self.wheel_distance = wheel_distance
        self.wheel_radius = wheel_radius
        self.max_wheel_speed = max_wheel_speed
        self.wheel_vel = np.zeros(2)  # [v_l, v_r]

        self.create_subscription(Twist, f'/env_{env_id}/robot/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Calculate wheel velocities
        v_l = (linear_x - (self.wheel_distance / 2.0) * angular_z) / self.wheel_radius
        v_r = (linear_x + (self.wheel_distance / 2.0) * angular_z) / self.wheel_radius
        v_l = np.clip(v_l, -self.max_wheel_speed, self.max_wheel_speed)
        v_r = np.clip(v_r, -self.max_wheel_speed, self.max_wheel_speed)  

        self.wheel_vel = np.array([v_l, v_r])      

class RobotDriverManager:
    def __init__(self, num_envs: int, wheel_distance=0.37558, wheel_radius=0.098, max_wheel_speed=500.0):
        self.num_envs = num_envs
        self.driver_nodes = [
            RobotDriverNode(i, wheel_distance, wheel_radius, max_wheel_speed) for i in range(num_envs)
        ]
        self.executor_thread = Thread(target=self.run_executor, daemon=True)
        self.executor_thread.start()

    def run_executor(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        for node in self.driver_nodes:
            executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for node in self.driver_nodes:
                node.destroy_node()

    def apply_all(self, robot_asset, joint_names: List[str]):
        velocities = []
        joint_ids = robot_asset.find_joints(joint_names, preserve_order=True)[0]
        for node in self.driver_nodes:
            v_l, v_r = node.wheel_vel
            velocities.append([v_l, v_l, v_r, v_r])
        
        robot_asset.set_joint_velocity_target(joint_ids = joint_ids, target = torch.tensor(velocities, dtype=torch.float32, device=robot_asset.device))

    def stop(self):
        self.executor_thread.join()