import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from threading import Thread

from autogen_bt_interface.srv import ChargingRequest

class BatteryNode(Node):
    def __init__(self, name: str, discharge_rate: float, charge_rate: float, robot_name: str = "robot", verbose = False):
        super().__init__(f'{name}_battery_node')

        self.initial_battery_level = 100.0 # Initial battery percentage
        self.battery_level = self.initial_battery_level
        self.is_charging = False
        self.name = name
        self.discharge_rate = discharge_rate
        self.charge_rate = charge_rate
        self.verbose = verbose

        if  len(robot_name) != 0 and robot_name[0] != '/':
            robot_name = '/' + robot_name

        # ROS2 Publishers
        self.battery_pub = self.create_publisher(Float32, f'{name}{robot_name}/battery_level', 10)

        # ROS2 Service Server
        self.create_service(ChargingRequest, f"{name}{robot_name}/charging_req", self.charge_callback)
        
        # Timer for battery update
        self.update_timer = self.create_timer(1.0, self.update_battery)

        self.get_logger().info(f"Battery node {name} initialized.")

    def charge_callback(self, req, res):
        self.is_charging = req.status.data
        if self.is_charging:
            if self.verbose: self.get_logger().info(f"{self.name}: Charging started.")
        else:
            if self.verbose: self.get_logger().info(f"{self.name}: Charging stopped.")

        return ChargingRequest.Response()

    def update_battery(self):
        if self.is_charging:
            self.battery_level = min(100.0, self.battery_level + self.charge_rate)
        else:
            self.battery_level = max(0.0, self.battery_level - self.discharge_rate)

        # Publish battery level
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)

        # self.get_logger().info(f"{self.name}: Battery level: {self.battery_level:.2f}%")

        if self.battery_level <= 0.0:
            if self.verbose: self.get_logger().warn(f"{self.name}: Battery empty! Please recharge.")

    def reset(self):
        self.battery_level = self.initial_battery_level
        self.is_charging = False

class BatteryManager:
    def __init__(self, battery_names, discharge_rate: float, charge_rate: float, robot_name: str = "robot"):
        """
        Initialize multiple battery nodes with shared charge and discharge rates.

        :param battery_names: A list of battery names (strings).
        :param discharge_rate: The shared discharge rate (% per second).
        :param charge_rate: The shared charge rate (% per second).
        """
    
        self.batteries = [
            BatteryNode(name, discharge_rate, charge_rate, robot_name) for name in battery_names
        ]

        self.executor_thread = Thread(target=self.run_executor, daemon=True)
        self.executor_thread.start()

    def reset(self):
        for battery in self.batteries:
            battery.reset()

    def run_executor(self):
        executor = rclpy.executors.MultiThreadedExecutor()
        for battery in self.batteries:
            executor.add_node(battery)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            for battery in self.batteries:
                battery.destroy_node()

    def stop(self):
        # Ensure the nodes shut down gracefully
        self.executor_thread.join()