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

        # Publish target transformation
        for j in range(5):
            base_node.publish_target_transform(scene[f"target_{j+1}"].data.root_state_w[i, :3], scene[f"target_{j+1}"].data.root_state_w[i, 3:7], i, j)
        

class SceneNode(Node):
    def __init__(self, num_envs):
        super().__init__('SceneNode')
        qos_profile = QoSProfile(depth=10)

        self.robot_joint_state_pub = []
        self.robot_transform_pub = []
        self.target_transform_pub = []

        temp = []

        for i in range(num_envs):
            self.robot_joint_state_pub.append(self.create_publisher(JointState, f'robot_{i}/joint_state', qos_profile))
            self.robot_transform_pub.append(self.create_publisher(TransformStamped, f'robot_{i}/tf', qos_profile))
            for j in range(5):
                temp.append(self.create_publisher(TransformStamped, f'env_{i}/target{j+1}/tf', qos_profile))
            self.target_transform_pub.append(temp)

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

    def publish_target_transform(self, base_pos, base_rot, env_ind, target_ind):
        trans = TransformStamped()
        trans.header.stamp = self.get_clock().now().to_msg()
        trans.header.frame_id = "world"

        trans.child_frame_id = f"Target_{target_ind}"
        trans.transform.translation.x = base_pos[0].item()
        trans.transform.translation.y = base_pos[1].item()
        trans.transform.translation.z = base_pos[2].item()
        trans.transform.rotation.x = base_rot[1].item()
        trans.transform.rotation.y = base_rot[2].item()
        trans.transform.rotation.z = base_rot[3].item()
        trans.transform.rotation.w = base_rot[0].item()

        self.target_transform_pub[env_ind][target_ind].publish(trans)
