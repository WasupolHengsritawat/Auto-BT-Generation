import py_trees
import py_trees_ros
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion
import numpy as np
import networkx as nx
import random
import math

# Utility Functions =======================================================================================================================================
def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def move_to_target(pid, robot_position, robot_orientation, target_position, cmd_vel_publisher, logger = None):
    """
    Move the robot towards the target position using a PID controller for angular velocity.

    :param robot_position: Tuple (x, y) of the robot's current position.
    :param robot_orientation: Quaternion representing the robot's orientation.
    :param target_position: Tuple (x, y) of the target position.
    :param cmd_vel_publisher: ROS2 publisher for /cmd_vel topic.
    :param pid: Instance of PIDController for angular velocity control.
    :param transform_stamped: TransformStamped message containing the timestamp.
    """
    status = False # Running  

    # Extract current position and target position
    x_robot, y_robot = robot_position
    x_target, y_target = target_position

    # Calculate target angle
    theta_target = math.atan2(y_target - y_robot, x_target - x_robot)

    # Get current orientation as yaw
    _, _, theta_current = euler_from_quaternion(robot_orientation)

    # Calculate angular error (normalize to [-pi, pi])
    angular_offset = np.pi
    angular_error = math.atan2(math.sin(theta_target - theta_current + angular_offset), math.cos(theta_target - theta_current + angular_offset))

    # Distance to the target
    distance_to_target = math.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)

    # Compute angular velocity using the PID controller
    angular_velocity = pid.compute(angular_error)

    # Proportional gain for linear velocity
    K_linear = 1.0
    linear_velocity = 0.0

    # If the angular error is small, move forward
    if abs(angular_error) < 0.1:
        linear_velocity = K_linear * distance_to_target
        # angular_velocity = 0.0
        if linear_velocity > 0.7:
            linear_velocity = 0.7

    # Stop if close to the target
    if distance_to_target < 0.2:
        linear_velocity = 0.0
        angular_velocity = 0.0
        pid.reset()  # Reset PID controller when goal is reached
        status = True # Finish

    # Publish the velocities
    twist_msg = Twist()
    twist_msg.linear.x = float(linear_velocity)
    twist_msg.angular.z = float(angular_velocity)

    if not(logger == None):
        logger.info(f'linear error  : {distance_to_target} angular error  : {angular_error}')
        logger.info(f'linear command: {linear_velocity} angular command: {angular_velocity}')

    cmd_vel_publisher.publish(twist_msg)

    return status

def find_nearest_node(position, graph):
    nearest_node = None
    min_distance = float('inf')  

    for node in graph.nodes:
        node_position = graph.nodes[node]['pos']  
        distance = math.sqrt((position[0] - node_position[0])**2 + (position[1] - node_position[1])**2)
        if distance < min_distance:
            min_distance = distance
            nearest_node = node

    return nearest_node

def robot_graph_adj_nodes(robot_graph, full_graph):
    adj_nodes = []
    for u in full_graph.nodes:
        if not(u in robot_graph.nodes):
            u_adj = [u_adj for u_adj in full_graph.neighbors(u)]
            for v in u_adj:
                if v in robot_graph and not(u in adj_nodes):
                    adj_nodes.append(u)
                    break
    return adj_nodes

def add_node_to_robot_graph(u, robot_graph, full_graph):
    u_adj = [u_adj for u_adj in full_graph.neighbors(u)]

    for n in robot_graph.nodes:
        if n in u_adj:
            robot_graph.add_node(u, pos=full_graph.nodes[u]['pos'])
            break

    for e in full_graph.edges:
        for v in robot_graph.nodes:
            if v in e and u in e and not(v == u):
                robot_graph.add_edge(u, v, weight=full_graph.edges[u,v]['weight'])
                break

class PIDController:
    def __init__(self, K_p = 1.0, K_i = 0.0, K_d = 0.0, output_limits=(None, None)):
        """
        Initialize the PID controller.

        :param K_p: Proportional gain.
        :param K_i: Integral gain.
        :param K_d: Derivative gain.
        :param output_limits: Tuple (min, max) to clamp the output.
        """
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d

        self.output_limits = output_limits

        self.prev_u = 0.0

        # Error history
        self.prev_error = 0.0
        self.prev2_error = 0.0
        self.integral = 0.0

    def reset(self):
        """Reset the controller state."""
        self.prev_error = 0.0
        self.prev2_error = 0.0
        self.prev_u = 0.0
        self.integral = 0.0

    def compute(self, error):
        """
        Compute the control output using the velocity form of the PID controller.

        :param error: Current error.
        :param current_time: Timestamp from TransformStamped (in nanoseconds).
        :return: Control output (u).
        """

        # Update control output incrementally
        u = self.prev_u + (self.K_p + self.K_i + self.K_d)*error - (self.K_p + 2*self.K_d)*self.prev_error + self.K_d*self.prev2_error

        # Apply limits
        min_output, max_output = self.output_limits
        if min_output is not None and max_output is not None:
            u = max(min(u, max_output), min_output)

        # Update error history
        self.prev2_error = self.prev_error
        self.prev_error = error
        self.prev_u = u

        return u

# Behavior Nodes ==========================================================================================================================================

class PatrolNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, full_graph, robot_graph, env_id):
        super().__init__(name)
        self.full_graph = full_graph
        self.robot_graph = robot_graph

        self.env_id = env_id

        self.robot_pos = None
        self.robot_rot = None
        self.cmd_vel_publisher = None

        self.path = []
        self.angular_controller = PIDController(K_p = 5, K_i = 2, output_limits=(-2.7,2.7))
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/robot_{self.env_id}/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/robot_{self.env_id}/cmd_vel", 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None:
            return py_trees.common.Status.RUNNING   
    
        if self.path == []: # Plan to go to adjacent unvisited area 

            # Use robot nearest node as the current node and check its unvisited adjacent nodes
            robot_current_node = find_nearest_node(self.robot_pos, self.robot_graph)
            robot_current_node_adj = [n for n in self.full_graph.neighbors(robot_current_node)]
            robot_current_node_adj_unvisited = [n for n in robot_current_node_adj if not(self.robot_graph.has_node(n))]
            
            if robot_current_node_adj_unvisited == []:  # All of the current adjacent node is visited -> use the graph adjacent node instead
                graph_adj_nodes = robot_graph_adj_nodes(self.robot_graph, self.full_graph)
                if graph_adj_nodes == []:
                    self.logger.info(f"The map has been finished.") 
                    return py_trees.common.Status.RUNNING 
                else:
                    final_target_node = np.random.choice(graph_adj_nodes)
            else: # Use one of the unvisited adjacent node as the target
                final_target_node = np.random.choice(robot_current_node_adj_unvisited)

            # Find the shortest path to the final target node
            self.path = nx.shortest_path(self.full_graph, source=robot_current_node, target=final_target_node, weight='weight')
        else: 
            # Execute the plan
            target_node = self.path[0]
            target_pos = self.full_graph.nodes[target_node]['pos']

            # Move to target
            status = move_to_target(self.angular_controller, self.robot_pos, self.robot_rot, target_pos, self.cmd_vel_publisher)
            self.logger.info(f"status: {'Finished' if status else f'Running to {target_node}'}")

            if status: # Finished moving to target
                # Update the robot graph
                add_node_to_robot_graph(target_node, self.robot_graph, self.full_graph)
                # remove the newly visited target from the path
                self.path = self.path[1:]

        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        self.logger.debug(f"{self.name}: terminate({new_status})")

class GoToChargerNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id):
        super().__init__(name)
        self.robot_graph = robot_graph

        self.env_id = env_id

        self.final_target_node = 1

        self.robot_pos = None
        self.robot_rot = None
        self.cmd_vel_publisher = None

        self.path = []
        self.angular_controller = PIDController(K_p = 5, K_i = 2, output_limits=(-2.7,2.7))
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/robot_{self.env_id}/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/robot_{self.env_id}/cmd_vel", 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None:
            return py_trees.common.Status.RUNNING   
    
        if self.path == []: # Plan to go to adjacent unvisited area 

            # Use robot nearest node as the current node and check its unvisited adjacent nodes
            robot_current_node = find_nearest_node(self.robot_pos, self.robot_graph)

            # Find the shortest path to the final target node
            self.path = nx.shortest_path(self.robot_graph, source=robot_current_node, target=self.final_target_node, weight='weight')
        else: 
            # Execute the plan
            target_node = self.path[0]
            target_pos = self.robot_graph.nodes[target_node]['pos']

            # Move to target
            status = move_to_target(self.angular_controller, self.robot_pos, self.robot_rot, target_pos, self.cmd_vel_publisher)
            self.logger.info(f"status: {'Finished' if status else f'Running to {target_node}'}")

            if status: # Finished moving to target
                # remove the newly visited target from the path
                self.path = self.path[1:]
                if target_node == self.final_target_node:
                    return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        self.logger.debug(f"{self.name}: terminate({new_status})")

class GoToSpawnNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id):
        super().__init__(name)
        self.robot_graph = robot_graph

        self.env_id = env_id

        self.final_target_node = 0

        self.robot_pos = None
        self.robot_rot = None
        self.cmd_vel_publisher = None

        self.path = []
        self.angular_controller = PIDController(K_p = 5, K_i = 2, output_limits=(-2.7,2.7))
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/robot_{self.env_id}/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/robot_{self.env_id}/cmd_vel", 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None:
            return py_trees.common.Status.RUNNING   
    
        if self.path == []: # Plan to go to adjacent unvisited area 

            # Use robot nearest node as the current node and check its unvisited adjacent nodes
            robot_current_node = find_nearest_node(self.robot_pos, self.robot_graph)

            # Find the shortest path to the final target node
            self.path = nx.shortest_path(self.robot_graph, source=robot_current_node, target=self.final_target_node, weight='weight')
        else: 
            # Execute the plan
            target_node = self.path[0]
            target_pos = self.robot_graph.nodes[target_node]['pos']

            # Move to target
            status = move_to_target(self.angular_controller, self.robot_pos, self.robot_rot, target_pos, self.cmd_vel_publisher)
            self.logger.info(f"status: {'Finished' if status else f'Running to {target_node}'}")

            if status: # Finished moving to target
                # remove the newly visited target from the path
                self.path = self.path[1:]
                if target_node == self.final_target_node:
                    return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        self.logger.debug(f"{self.name}: terminate({new_status})")

class FindTargetNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_graph, robot_graph, env_id):
        super().__init__(name)
        self.target_graph = target_graph
        self.robot_graph = robot_graph

        self.env_id = env_id

        self.target_pos = [None, None, None, None, None]
        self.target_rot = [None, None, None, None, None]
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.target_1_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/target1/tf", self.target_1_trans_callback, 10)
        self.target_2_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/target2/tf", self.target_2_trans_callback, 10)
        self.target_3_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/target3/tf", self.target_3_trans_callback, 10)
        self.target_4_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/target4/tf", self.target_4_trans_callback, 10)
        self.target_5_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/target5/tf", self.target_5_trans_callback, 10)

        return True
    
    def target_1_trans_callback(self, msg):
        self.target_pos[0] = [msg.transform.translation.x, msg.transform.translation.y]
        self.target_rot[0] = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def target_2_trans_callback(self, msg):
        self.target_pos[1] = [msg.transform.translation.x, msg.transform.translation.y]
        self.target_rot[1] = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
    
    def target_3_trans_callback(self, msg):
        self.target_pos[2] = [msg.transform.translation.x, msg.transform.translation.y]
        self.target_rot[2] = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
    
    def target_4_trans_callback(self, msg):
        self.target_pos[3] = [msg.transform.translation.x, msg.transform.translation.y]
        self.target_rot[3] = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
    
    def target_5_trans_callback(self, msg):
        self.target_pos[4] = [msg.transform.translation.x, msg.transform.translation.y]
        self.target_rot[4] = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
            
    def update(self):
        """
        Main behavior logic.
        """
        if None in self.target_pos or None in self.target_rot:
            return py_trees.common.Status.RUNNING   
    
        target_spawn_pool = [[ 14.0, -4.0], # 20 - target  #1
                             [ 22.0, -1.0], # 21 - target  #2
                             [ 19.5,  7.7], # 22 - target  #3
                             [ 20.5, 13.0], # 23 - target  #4
                             [ 23.0, 21.0], # 24 - target  #5
                             [  9.0, 24.5], # 25 - target  #6
                             [ -2.0, 23.2], # 26 - target  #7
                             [ -1.5, 13.3], # 27 - target  #8
                             [ -2.6,  7.7], # 28 - target  #9
                             [  8.0,  6.7]] # 29 - target #10
        
        valid_target = []

        for target_pos in self.target_pos:
            for i in range(len(target_spawn_pool)):
                if euclidean_distance(target_pos, target_spawn_pool[i]) < 0.2:
                    valid_target.append(20+i)
                    break
        
        for u in valid_target:
            add_node_to_robot_graph(u, self.robot_graph, self.target_graph)
            self.logger.info(f'Add target node {u} to the robot graph -> robot graph size: {self.robot_graph.number_of_nodes()}')

        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        self.logger.debug(f"{self.name}: terminate({new_status})")
# =========================================================================================================================================================

class MoveNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, target, env_id):
        super().__init__(name)
        self.env_id = env_id

        self.target = target

        self.robot_pos = None
        self.robot_rot = None
        self.current_time = None
        
        self.robot_trans_subscriber = None
        self.cmd_vel_publisher = None

        self.angular_controller = PIDController(K_p = 2.0, K_i = 0.5)
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
            self.logger.info(f"Successfully setup {self.name}")
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/robot_{self.env_id}/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/robot_{self.env_id}/cmd_vel", 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
        self.current_time = msg.header.stamp.nanosec

    # def move_to_target(self, robot_position, robot_orientation, target_position):
    #     """
    #     Move the robot towards the target position using a PID controller for angular velocity.

    #     :param robot_position: Tuple (x, y) of the robot's current position.
    #     :param robot_orientation: Quaternion representing the robot's orientation.
    #     :param target_position: Tuple (x, y) of the target position.
    #     :param cmd_vel_publisher: ROS2 publisher for /cmd_vel topic.
    #     :param pid: Instance of PIDController for angular velocity control.
    #     :param transform_stamped: TransformStamped message containing the timestamp.
    #     """
    #     # Extract current position and target position
    #     x_robot, y_robot = robot_position
    #     x_target, y_target = target_position

    #     # Calculate target angle
    #     theta_target = math.atan2(y_target - y_robot, x_target - x_robot)

    #     # Get current orientation as yaw
    #     _, _, theta_current = euler_from_quaternion(robot_orientation)

    #     # Calculate angular error (normalize to [-pi, pi])
    #     angular_offset = np.pi
    #     angular_error = math.atan2(math.sin(theta_target - theta_current + angular_offset), math.cos(theta_target - theta_current + angular_offset))

    #     # Distance to the target
    #     distance_to_target = math.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)
        
    #     self.logger.info(f'linear error: {distance_to_target} angular error: {angular_error}')

    #     # Compute angular velocity using the PID controller
    #     angular_velocity = self.angular_controller.compute(angular_error)

    #     # Proportional gain for linear velocity
    #     K_linear = 0.5
    #     linear_velocity = 0.0

    #     # If the angular error is small, move forward
    #     if abs(angular_error) < 0.1:
    #         linear_velocity = K_linear * distance_to_target
    #         # angular_velocity = 0.0
    #         if linear_velocity > 1.0:
    #             linear_velocity = 1.0

    #     # Stop if close to the target
    #     if distance_to_target < 0.1:
    #         linear_velocity = 0.0
    #         angular_velocity = 0.0
    #         self.angular_controller.reset()  # Reset PID controller when goal is reached

    #     # Publish the velocities
    #     twist_msg = Twist()
    #     twist_msg.linear.x = linear_velocity
    #     twist_msg.angular.z = angular_velocity
    #     self.logger.info(f"publish linear: {linear_velocity} angular: {angular_velocity}")
    #     self.cmd_vel_publisher.publish(twist_msg)

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None or self.target is None:
            return py_trees.common.Status.RUNNING
    
        move_to_target(self.angular_controller, self.robot_pos, self.robot_rot, self.target, self.cmd_vel_publisher)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        self.logger.debug(f"{self.name}: terminate({new_status})")