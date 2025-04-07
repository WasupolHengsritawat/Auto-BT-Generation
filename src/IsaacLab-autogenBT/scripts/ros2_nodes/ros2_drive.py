import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from threading import Thread
import numpy as np
import torch
from typing import List

class RobotDriverManagerNode(Node):
    def __init__(self, num_envs: int, wheel_distance: float=0.37558, wheel_radius: float=0.098, max_wheel_speed: float=500.0):
        """
        A single ROS2 node to manage wheel velocities for multiple robot environments.

        :param num_envs: Number of simulation environments.
        :param wheel_distance: Distance between wheels.
        :param wheel_radius: Radius of the wheels.
        :param max_wheel_speed: Maximum allowed speed per wheel.
        """
        super().__init__('robot_driver_manager_node')

        self.num_envs = num_envs
        self.wheel_distance = wheel_distance
        self.wheel_radius = wheel_radius
        self.max_wheel_speed = max_wheel_speed

        self.wheel_velocities = np.zeros((num_envs, 2))  # shape = (N, 2) for [v_l, v_r]

        for env_id in range(num_envs):
            topic = f'/env_{env_id}/robot/cmd_vel'
            self.create_subscription(Twist, topic, self.make_callback(env_id), 10)

        self.get_logger().info(f"RobotDriverManagerNode initialized for {num_envs} environments.")

    def make_callback(self, env_id: int):
        """
        Factory function to generate a callback for each environment.

        :param env_id: Index of the environment.
        """
        def callback(msg: Twist):
            linear_x = msg.linear.x
            angular_z = msg.angular.z

            # Compute individual wheel velocities
            v_l = (linear_x - (self.wheel_distance / 2.0) * angular_z) / self.wheel_radius
            v_r = (linear_x + (self.wheel_distance / 2.0) * angular_z) / self.wheel_radius

            # Clip to max speed
            v_l = np.clip(v_l, -self.max_wheel_speed, self.max_wheel_speed)
            v_r = np.clip(v_r, -self.max_wheel_speed, self.max_wheel_speed)

            self.wheel_velocities[env_id] = [v_l, v_r]

        return callback

    def get_all_wheel_velocities(self):
        """
        :return: (N, 2) ndarray of wheel velocities for all environments.
        """
        return self.wheel_velocities.copy()
    
    def apply_all(self, robot_asset, joint_names: List[str]):
        """
        Apply wheel velocities to all robots in simulation.

        :param robot_asset: IsaacLab robot asset object.
        :param joint_names: Ordered list of joint names: [FL, BL, FR, BR].
        """
        joint_ids = robot_asset.find_joints(joint_names, preserve_order=True)[0]
        velocities = self.get_all_wheel_velocities()

        # Set velocity for [FL, BL, FR, BR] for each robot
        formatted_velocities = torch.tensor(
            [[v_l, v_l, v_r, v_r] for v_l, v_r in velocities],
            dtype=torch.float32,
            device=robot_asset.device
        )

        robot_asset.set_joint_velocity_target(joint_ids=joint_ids, target=formatted_velocities)


class RobotDriverManager:
    def __init__(self, num_envs: int, wheel_distance=0.37558, wheel_radius=0.098, max_wheel_speed=500.0):
        """
        Wrapper class to run RobotDriverManagerNode in background thread and apply wheel commands to robot.

        :param num_envs: Number of environments.
        :param wheel_distance: Distance between robot wheels.
        :param wheel_radius: Radius of wheels.
        :param max_wheel_speed: Max speed of each wheel.
        """
        self.node = RobotDriverManagerNode(num_envs, wheel_distance, wheel_radius, max_wheel_speed)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def apply(self, robot_asset, joint_names: List[str]):
        """
        Apply wheel velocities to all robots in simulation.

        :param robot_asset: IsaacLab robot asset object.
        :param joint_names: Ordered list of joint names: [FL, BL, FR, BR].
        """
        joint_ids = robot_asset.find_joints(joint_names, preserve_order=True)[0]
        velocities = self.node.get_all_wheel_velocities()

        # Set velocity for [FL, BL, FR, BR] for each robot
        formatted_velocities = torch.tensor(
            [[v_l, v_l, v_r, v_r] for v_l, v_r in velocities],
            dtype=torch.float32,
            device=robot_asset.device
        )

        robot_asset.set_joint_velocity_target(joint_ids=joint_ids, target=formatted_velocities)

    def stop(self):
        """
        Stop ROS2 executor and thread cleanly.
        """
        self.executor.shutdown()
        self.node.destroy_node()
        self.executor_thread.join()
