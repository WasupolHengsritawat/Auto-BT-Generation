import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from threading import Thread
from rclpy.executors import MultiThreadedExecutor

from autogen_bt_interface.srv import ChargingRequest

class BatteryManagerNode(Node):
    def __init__(self, battery_names, discharge_rate: float, charge_rate: float, robot_name: str = "robot", verbose=False):
        """
        A single ROS2 node to manage multiple virtual batteries for different environments.

        :param battery_names: List of battery names (strings) for each environment.
        :param discharge_rate: Battery drain rate (% per second).
        :param charge_rate: Battery charging rate (% per second).
        :param robot_name: Prefix used in topic/service naming (defaults to "robot").
        :param verbose: Whether to print debug logs.
        """
        super().__init__('battery_manager_node')

        self.battery_levels = {}     # Dictionary to store battery level for each battery
        self.is_charging = {}        # Dictionary to track charging state per battery
        self.verbose = verbose
        self.charge_rate = charge_rate
        self.discharge_rate = discharge_rate

        if len(robot_name) != 0 and robot_name[0] != '/':
            robot_name = '/' + robot_name
        self.robot_name = robot_name

        # ROS2 Publishers and Service Servers for each battery
        self.battery_publishers = {}
        self.service_servers = {}

        for name in battery_names:
            full_name = f"{name}{self.robot_name}"
            self.battery_levels[name] = 100.0  # Initial battery percentage
            self.is_charging[name] = False

            # Create publisher for battery level
            self.battery_publishers[name] = self.create_publisher(Float32, f'{full_name}/battery_level', 10)

            # Create service for charging control
            self.service_servers[name] = self.create_service(
                ChargingRequest,
                f"{full_name}/charging_req",
                self.make_charge_callback(name)
            )

            self.get_logger().info(f"Battery for {name} initialized.")

        # Timer to update all battery levels every second
        self.update_timer = self.create_timer(1.0, self.update_batteries)

    def make_charge_callback(self, name):
        """
        Factory method to create a unique charging callback for each battery.

        :param name: Battery name to bind with the callback.
        :return: Callback function for charging service.
        """
        def callback(req, res):
            self.is_charging[name] = req.status.data
            if self.verbose:
                state = "started" if req.status.data else "stopped"
                self.get_logger().info(f"{name}: Charging {state}.")
            return ChargingRequest.Response()
        return callback

    def update_batteries(self):
        """
        Updates battery levels based on charging status, and publishes the new battery levels.
        """
        for name in self.battery_levels.keys():
            if self.is_charging[name]:
                self.battery_levels[name] = min(100.0, self.battery_levels[name] + self.charge_rate)
            else:
                self.battery_levels[name] = max(0.0, self.battery_levels[name] - self.discharge_rate)

            # Publish updated battery level
            msg = Float32()
            msg.data = self.battery_levels[name]
            self.battery_publishers[name].publish(msg)

            if self.battery_levels[name] <= 0.0 and self.verbose:
                self.get_logger().warn(f"{name}: Battery empty! Please recharge.")

    def reset(self):
        """
        Resets all batteries to full charge and stops charging.
        """
        for name in self.battery_levels.keys():
            self.battery_levels[name] = 100.0
            self.is_charging[name] = False


class BatteryManager:
    def __init__(self, battery_names, discharge_rate: float, charge_rate: float, robot_name: str = "robot", verbose=False):
        """
        Class to run the BatteryManagerNode in a background thread using MultiThreadedExecutor.

        :param battery_names: List of environment battery names.
        :param discharge_rate: Discharge rate (% per second).
        :param charge_rate: Charge rate (% per second).
        :param robot_name: Robot name prefix for topics/services.
        :param verbose: Whether to show logs.
        """
        self.node = BatteryManagerNode(battery_names, discharge_rate, charge_rate, robot_name, verbose)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)

        self.executor_thread = Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def reset(self):
        """
        Resets all batteries to full and stops charging.
        """
        self.node.reset()

    def stop(self):
        """
        Stops the ROS2 node and background thread cleanly.
        """
        self.executor.shutdown()
        self.node.destroy_node()
        self.executor_thread.join()
