from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped


def pub_scene_data(num_envs, base_node, scene):
    for i in range(num_envs):
        # publish ros2 info
        base_node.publish_transform(scene["robot"].data.root_state_w[i, :3], scene["robot"].data.root_state_w[i, 3:7], i)

class RobotBaseNode(Node):
    def __init__(self, num_envs):
        super().__init__('RobotBaseNode')
        qos_profile = QoSProfile(depth=10)

        self.robot_transform_pub = []
        self.target_transform_pub = []
        temp = []

        for i in range(num_envs):
            self.robot_transform_pub.append(self.create_publisher(TransformStamped, f'robot_{i}/tf', qos_profile))
            for j in range(5):
                temp.append(self.create_publisher(TransformStamped, f'env_{i}/target_{j+1}_tf', qos_profile))
            self.robot_transform_pub.append(temp)

    def publish_transform(self, base_pos, base_rot, env_ind, type):
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = "odom"

        if type == 'robot':
            trans.child_frame_id = f"robot_{env_ind}/base_link"
        elif type == 'target_1':
            trans.child_frame_id = f"robot_{env_ind}/base_link"

        trans.transform.translation.x = base_pos[0].item()
        trans.transform.translation.y = base_pos[1].item()
        trans.transform.translation.z = base_pos[2].item()
        trans.transform.rotation.x = base_rot[1].item()
        trans.transform.rotation.y = base_rot[2].item()
        trans.transform.rotation.z = base_rot[3].item()
        trans.transform.rotation.w = base_rot[0].item()

        self.robot_transform_pub[env_ind].publish(trans)
