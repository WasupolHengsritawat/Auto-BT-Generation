import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from threading import Thread


class BatteryNode(Node):
    def __init__(self, name: str, discharge_rate: float, charge_rate: float):
        super().__init__(f'{name}_battery_node')

        self.battery_level = 100.0  # Initial battery percentage
        self.is_charging = False
        self.name = name
        self.discharge_rate = discharge_rate
        self.charge_rate = charge_rate

        # Publishers and Subscribers
        self.battery_pub = self.create_publisher(Float32, f'{name}/battery_level', 10)
        self.charge_sub = self.create_subscription(Bool, f'{name}/charge_request', self.charge_callback, 10)

        # Timer for battery update
        self.update_timer = self.create_timer(1.0, self.update_battery)

        self.get_logger().info(f"Battery node {name} initialized.")

    def charge_callback(self, msg: Bool):
        self.is_charging = msg.data
        if self.is_charging:
            self.get_logger().info(f"{self.name}: Charging started.")
        else:
            self.get_logger().info(f"{self.name}: Charging stopped.")

    def update_battery(self):
        if self.is_charging:
            self.battery_level = min(100.0, self.battery_level + self.charge_rate)
        else:
            self.battery_level = max(0.0, self.battery_level - self.discharge_rate)

        # Publish battery level
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)

        self.get_logger().info(f"{self.name}: Battery level: {self.battery_level:.2f}%")

        if self.battery_level <= 0.0:
            self.get_logger().warn(f"{self.name}: Battery empty! Please recharge.")


class BatteryManager:
    def __init__(self, battery_names, discharge_rate: float, charge_rate: float):
        """
        Initialize multiple battery nodes with shared charge and discharge rates.

        :param battery_names: A list of battery names (strings).
        :param discharge_rate: The shared discharge rate (% per second).
        :param charge_rate: The shared charge rate (% per second).
        """
    
        self.batteries = [
            BatteryNode(name, discharge_rate, charge_rate) for name in battery_names
        ]

        self.executor_thread = Thread(target=self.run_executor, daemon=True)
        self.executor_thread.start()

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
            rclpy.shutdown()

    def stop(self):
        # Ensure the nodes shut down gracefully
        rclpy.shutdown()
        self.executor_thread.join()