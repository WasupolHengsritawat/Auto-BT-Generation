from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped, Point
from sensor_msgs.msg import JointState

def pub_scene_data(num_envs, base_node, scene):

    for env_id in range(num_envs):
        # Publish environment origin
        base_node.publish_env_origin(scene.env_origins[env_id,:], env_id)

        # Publish robot joint state
        base_node.publish_robot_joint_state(scene["robot"].data.joint_names, scene["robot"].data.joint_pos[env_id], env_id)
        
        # Publish robot transformation
        base_node.publish_robot_transform(scene["robot"].data.root_state_w[env_id, :3], scene["robot"].data.root_state_w[env_id, 3:7], env_id)

class SceneNode(Node):
    def __init__(self, num_envs):
        super().__init__('SceneNode')
        qos_profile = QoSProfile(depth=10)

        self.env_origin_pub = []
        self.robot_joint_state_pub = []
        self.robot_transform_pub = []

        for i in range(num_envs):
            self.env_origin_pub.append(self.create_publisher(Point, f'env_{i}/origin', qos_profile))
            self.robot_joint_state_pub.append(self.create_publisher(JointState, f'env_{i}/robot/joint_state', qos_profile))
            self.robot_transform_pub.append(self.create_publisher(TransformStamped, f'env_{i}/robot/tf', qos_profile))
    
    def publish_env_origin(self, env_origin, env_ind):
        msg = Point()
        msg.x = float(env_origin[0])
        msg.y = float(env_origin[1])
        msg.z = float(env_origin[2])

        self.env_origin_pub[env_ind].publish(msg)

    def publish_robot_joint_state(self, robot_joint_names, robot_joint_state, env_ind):
        msg = JointState()
        msg.name = robot_joint_names
        msg.position = robot_joint_state.tolist()

        self.robot_joint_state_pub[env_ind].publish(msg)
    
    def publish_robot_transform(self, base_pos, base_rot, env_ind):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = f"robot_{env_ind}"
        msg.transform.translation.x = base_pos[0].item()
        msg.transform.translation.y = base_pos[1].item()
        msg.transform.translation.z = base_pos[2].item()
        msg.transform.rotation.x = base_rot[1].item()
        msg.transform.rotation.y = base_rot[2].item()
        msg.transform.rotation.z = base_rot[3].item()
        msg.transform.rotation.w = base_rot[0].item()

        self.robot_transform_pub[env_ind].publish(msg)