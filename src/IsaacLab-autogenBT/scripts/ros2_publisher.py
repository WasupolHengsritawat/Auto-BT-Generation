from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

def pub_scene_data(num_envs, base_node, scene):

    for i in range(num_envs):
        # Publish robot joint state
        base_node.publish_robot_joint_state(scene["robot"].data.joint_names, scene["robot"].data.joint_pos[i], i)
        
        # Publish robot transformation
        base_node.publish_robot_transform(scene["robot"].data.root_state_w[i, :3], scene["robot"].data.root_state_w[i, 3:7], i)

class SceneNode(Node):
    def __init__(self, num_envs):
        super().__init__('SceneNode')
        qos_profile = QoSProfile(depth=10)

        self.robot_joint_state_pub = []
        self.robot_transform_pub = []

        for i in range(num_envs):
            self.robot_joint_state_pub.append(self.create_publisher(JointState, f'env_{i}/robot/joint_state', qos_profile))
            self.robot_transform_pub.append(self.create_publisher(TransformStamped, f'env_{i}/robot/tf', qos_profile))

    def publish_robot_joint_state(self, robot_joint_names, robot_joint_state, env_ind):
        joint_state = JointState()
        joint_state.name = robot_joint_names
        joint_state.position = robot_joint_state.tolist()

        self.robot_joint_state_pub[env_ind].publish(joint_state)
    
    def publish_robot_transform(self, base_pos, base_rot, env_ind):
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = "world"
        trans.child_frame_id = f"robot_{env_ind}"
        trans.transform.translation.x = base_pos[0].item()
        trans.transform.translation.y = base_pos[1].item()
        trans.transform.translation.z = base_pos[2].item()
        trans.transform.rotation.x = base_rot[1].item()
        trans.transform.rotation.y = base_rot[2].item()
        trans.transform.rotation.z = base_rot[3].item()
        trans.transform.rotation.w = base_rot[0].item()

        self.robot_transform_pub[env_ind].publish(trans)