from py_trees_ros_interfaces.msg import BehaviourTree as BTMsg
from unique_identifier_msgs.msg import UUID
import rclpy

class BTStatusTracker:
    def __init__(self, node: rclpy.node.Node, env_id: int = 0):
        self.root_status = None
        topic_name = f"/env_{env_id}_tree/snapshots"
        self.sub = node.create_subscription(
            BTMsg,
            topic_name,
            self.snapshot_callback,
            10
        )

    def snapshot_callback(self, msg: BTMsg):
        for behavior in msg.behaviours:
            if behavior.parent_id.uuid == [0]*16:  # root has no parent
                self.root_status = behavior.status  # 0=INVALID, 1=RUNNING, 2=SUCCESS, 3=FAILURE
                break

    def get_status_text(self):
        status_map = {
            0: "INVALID",
            1: "RUNNING",
            2: "SUCCESS",
            3: "FAILURE"
        }
        return status_map.get(self.root_status, "UNKNOWN")
