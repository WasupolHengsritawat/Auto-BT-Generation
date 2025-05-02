import py_trees
import py_trees_ros
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import q2r
from geometry_msgs.msg import Twist, PointStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float32, Bool, UInt8
from tf_transformations import euler_from_quaternion
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import networkx as nx
import random
import math
import sys 

from autogen_bt_interface.srv import ChargingRequest, PickingRequest

# Utility Functions =======================================================================================================================================
def euclidean_distance(coord1, coord2):
    return math.sqrt((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)

def trapezoidal_velocity(distance_to_target, max_velocity, acceleration, deceleration, current_velocity, delta_time):
    """
    Calculate the linear velocity using a trapezoidal profile.

    :param distance_to_target: Remaining distance to the target.
    :param max_velocity: Maximum allowable velocity.
    :param acceleration: Acceleration rate.
    :param deceleration: Deceleration rate.
    :param current_velocity: Current velocity of the robot.
    :param delta_time: Time step for updating velocity.
    :return: Updated velocity.
    """
    # Compute braking distance based on deceleration
    braking_distance = (current_velocity**2) / (2 * deceleration)
    
    if distance_to_target <= braking_distance:
        # Decelerate if within braking distance
        new_velocity = max(0, current_velocity - deceleration * delta_time)
    elif current_velocity < max_velocity:
        # Accelerate if below max velocity
        new_velocity = min(max_velocity, current_velocity + acceleration * delta_time)
    else:
        # Maintain max velocity
        new_velocity = max_velocity

    return new_velocity

def move_to_target(pid, robot_position, robot_orientation, target_position, cmd_vel_publisher, logger=None, delta_time=0.1, tol=0.2):
    """
    Move the robot towards the target position using a PID controller for angular velocity
    and trapezoidal velocity profile for linear velocity.

    :param robot_position: Tuple (x, y) of the robot's current position.
    :param robot_orientation: Quaternion representing the robot's orientation.
    :param target_position: Tuple (x, y) of the target position.
    :param cmd_vel_publisher: ROS2 publisher for /cmd_vel topic.
    :param pid: Instance of PIDController for angular velocity control.
    :param logger: Optional logger for debugging.
    :param delta_time: Time step for velocity updates (default: 0.1 seconds).
    """
    status = False  # Running

    # Constants for the trapezoidal velocity profile
    max_velocity = 2.0  # Maximum linear velocity
    acceleration = 2.0  # Acceleration rate
    deceleration = 0.7  # Deceleration rate

    # Initialize static variable for current velocity
    if not hasattr(move_to_target, "current_velocity"):
        move_to_target.current_velocity = 0.0

    # Extract current position and target position
    x_robot, y_robot = robot_position
    x_target, y_target = target_position

    # Calculate target angle
    theta_target = math.atan2(y_target - y_robot, x_target - x_robot)

    # Get current orientation as yaw
    _, _, theta_current = euler_from_quaternion(robot_orientation)

    # Calculate angular error (normalize to [-pi, pi])
    angular_offset = np.pi
    angular_error = math.atan2(
        math.sin(theta_target - theta_current + angular_offset),
        math.cos(theta_target - theta_current + angular_offset),
    )

    # Distance to the target
    distance_to_target = math.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)

    # Compute angular velocity using the PID controller
    angular_velocity = pid.compute(angular_error)

    # Handle linear motion and trajectory reset
    linear_velocity = 0.0
    if abs(angular_error) < 0.1:  # Allow movement if angular error is small
        # Recompute trajectory and use trapezoidal velocity
        move_to_target.current_velocity = trapezoidal_velocity(
            distance_to_target,
            max_velocity,
            acceleration,
            deceleration,
            move_to_target.current_velocity,
            delta_time,
        )
        linear_velocity = move_to_target.current_velocity
    else:
        # Stop linear motion due to excessive angular error and reset velocity
        move_to_target.current_velocity = 0.0

    # Stop if close to the target
    if distance_to_target < tol:
        linear_velocity = 0.0
        angular_velocity = 0.0
        pid.reset()  # Reset PID controller when goal is reached
        move_to_target.current_velocity = 0.0  # Reset static velocity
        status = True  # Finish

    # Publish the velocities
    twist_msg = Twist()
    twist_msg.linear.x = float(linear_velocity)
    twist_msg.angular.z = float(angular_velocity)

    if logger is not None:
        logger.info(f"Linear error  : {distance_to_target}, Angular error  : {angular_error}")
        logger.info(f"Linear command: {linear_velocity}, Angular command: {angular_velocity}")

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
    def __init__(self, K_p = 5.0, K_i = 0.0, K_d = 0.0, output_limits=(None, None)):
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
    def __init__(self, name, full_graph, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.full_graph = full_graph
        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None

        self.robot_pos = None
        self.robot_rot = None
        self.cmd_vel_publisher = None

        self.path = []
        self.angular_controller = PIDController(K_p = 5, K_i = 2, output_limits=(-2,2))
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.battery_subscriber = self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/env_{self.env_id}/robot/cmd_vel", 10)

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None or self.battery_level is None:
            return py_trees.common.Status.RUNNING   
        
        if self.battery_level < 0.1:
            return py_trees.common.Status.FAILURE
        
        if set(self.full_graph.nodes).issubset(set(self.robot_graph.nodes)):
            return py_trees.common.Status.SUCCESS
    
        if self.path == []: # Plan to go to adjacent unvisited area 

            # Use robot nearest node as the current node and check its unvisited adjacent nodes
            robot_current_node = find_nearest_node(self.robot_pos, self.robot_graph)
            robot_current_node_adj = [n for n in self.full_graph.neighbors(robot_current_node)]
            robot_current_node_adj_unvisited = [n for n in robot_current_node_adj if not(self.robot_graph.has_node(n))]
            
            if robot_current_node_adj_unvisited == []:  # All of the current adjacent node is visited -> use the graph adjacent node instead
                graph_adj_nodes = robot_graph_adj_nodes(self.robot_graph, self.full_graph)
                if graph_adj_nodes == []:
                    if self.verbose: self.logger.info(f"The map has been finished.") 
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
            if self.verbose: self.logger.info(f"status: {'Finished' if status else f'Running to {target_node}'}")

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
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)

        self.path = []
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class FindTargetNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, target_graph, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="target_in_robot_graph", access=py_trees.common.Access.WRITE)
        
        self.target_graph = target_graph
        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None

        self.target_pos = [None, None, None, None, None]

        self.target_spawn_pool_size = 10

        self.is_counted = []
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target0/tf", self.target_0_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target1/tf", self.target_1_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target2/tf", self.target_2_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target3/tf", self.target_3_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target4/tf", self.target_4_trans_callback, 10)

        self.blackboard.target_in_robot_graph = 0

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data
    
    def target_0_trans_callback(self, msg):
        self.target_pos[0] = [msg.point.x, msg.point.y]

    def target_1_trans_callback(self, msg):
        self.target_pos[1] = [msg.point.x, msg.point.y]
    
    def target_2_trans_callback(self, msg):
        self.target_pos[2] = [msg.point.x, msg.point.y]
    
    def target_3_trans_callback(self, msg):
        self.target_pos[3] = [msg.point.x, msg.point.y]
    
    def target_4_trans_callback(self, msg):
        self.target_pos[4] = [msg.point.x, msg.point.y]
            
    def update(self):
        """
        Main behavior logic.
        """
        if None in self.target_pos or self.battery_level is None:
            return py_trees.common.Status.RUNNING  

        if self.battery_level < 0.1:
            return py_trees.common.Status.FAILURE 
        
        target_spawn_pool = list(nx.get_node_attributes(self.target_graph,'pos').values())[len(self.target_graph.nodes)-self.target_spawn_pool_size:]
        
        valid_target = []

        for target_pos in self.target_pos:
            for i in range(len(target_spawn_pool)):
                if euclidean_distance(target_pos, target_spawn_pool[i]) < 0.5:
                    valid_target.append(20+i)
                    break
        
        for u in valid_target:
            add_node_to_robot_graph(u, self.robot_graph, self.target_graph)
            if u in self.robot_graph.nodes:
                if not(u in self.is_counted):
                    if self.verbose: self.logger.info(f'Add target node {u} to the robot graph -> robot graph size: {self.robot_graph.number_of_nodes()}')
                    self.blackboard.target_in_robot_graph = self.blackboard.target_in_robot_graph + 1
                    self.is_counted.append(u)
            else:
                if u in self.is_counted:
                    self.is_counted.remove(u)
            

        return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class GoToChargerNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None

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
        
        self.battery_subscriber = self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/env_{self.env_id}/robot/cmd_vel", 10)

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None or self.battery_level is None:
            return py_trees.common.Status.RUNNING   
        
        if self.battery_level < 0.1:
            return py_trees.common.Status.FAILURE
    
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
            if self.verbose: self.logger.info(f"status: {'Finished' if status else f'Running to {target_node}'}")

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
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)

        self.path = []
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class GoToSpawnNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None

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
        
        self.battery_subscriber = self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/env_{self.env_id}/robot/cmd_vel", 10)

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None or self.battery_level is None:
            return py_trees.common.Status.RUNNING  

        if self.battery_level < 0.1:
            return py_trees.common.Status.FAILURE 
    
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
            if self.verbose: self.logger.info(f"status: {'Finished' if status else f'Running to {target_node}'}")

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
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)

        self.path = []
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class GoToNearestTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)

        try:
            self.blackboard.register_key(key="target_in_robot_graph", access=py_trees.common.Access.READ)
            self.blackboard.target_in_robot_graph
        except:
            self.blackboard.register_key(key="target_in_robot_graph", access=py_trees.common.Access.WRITE)
            self.blackboard.target_in_robot_graph = 0

        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose

        self.path = []
        self.angular_controller = PIDController(K_p = 5, K_i = 2, output_limits=(-2.7,2.7))

        self.robot_pos = None
        self.robot_rot = None
        self.battery_level = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.battery_subscriber = self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/env_{self.env_id}/robot/cmd_vel", 10)

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None or self.battery_level is None:
            return py_trees.common.Status.RUNNING   
        
        if self.battery_level < 0.1:
            return py_trees.common.Status.FAILURE  
        
        try:
            if self.blackboard.target_in_robot_graph == 0:
                return py_trees.common.Status.FAILURE  
        except:
            return py_trees.common.Status.FAILURE
    
        if self.path == []: # Plan to go to adjacent unvisited area 

            # Use robot nearest node as the current node and check its unvisited adjacent nodes
            robot_current_node = find_nearest_node(self.robot_pos, self.robot_graph)

            target_in_graph = [u for u in range(20,30) if u in self.robot_graph]

            nearest_target_distance = np.inf
            for valid_target in target_in_graph:
                valid_target_distance = nx.shortest_path_length(self.robot_graph, source=robot_current_node, target=valid_target, weight='weight')
                if valid_target_distance < nearest_target_distance:
                    self.final_target_node = valid_target
                    nearest_target_distance = valid_target_distance

            # Find the shortest path to the final target node
            self.path = nx.shortest_path(self.robot_graph, source=robot_current_node, target=self.final_target_node, weight='weight')
        else: 
            # Execute the plan
            target_node = self.path[0]
            target_pos = self.robot_graph.nodes[target_node]['pos']

            # Move to target
            status = move_to_target(self.angular_controller, self.robot_pos, self.robot_rot, target_pos, self.cmd_vel_publisher)
            if self.verbose: self.logger.info(f"status: {'Finished' if status else f'Running to {target_node}'}")

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
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(twist_msg)

        self.path = []
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None

        self.robot_pos = None
        self.robot_rot = None

        self.target_pos = [None] * 5

        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_in_robot_graph", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_in_robot_graph", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # ROS2 Message Setup -----------------------------------------------------------------------------------
        # Subscriber 
        self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)
        self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)

        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target0/tf", self.target_0_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target1/tf", self.target_1_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target2/tf", self.target_2_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target3/tf", self.target_3_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target4/tf", self.target_4_trans_callback, 10)

        # Service Client
        self.picking_request_client = self.node.create_client(PickingRequest, f"/env_{self.env_id}/picking_req")

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data

    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]
    
    def target_0_trans_callback(self, msg):
        self.target_pos[0] = [msg.point.x, msg.point.y]

    def target_1_trans_callback(self, msg):
        self.target_pos[1] = [msg.point.x, msg.point.y]
    
    def target_2_trans_callback(self, msg):
        self.target_pos[2] = [msg.point.x, msg.point.y]
    
    def target_3_trans_callback(self, msg):
        self.target_pos[3] = [msg.point.x, msg.point.y]
    
    def target_4_trans_callback(self, msg):
        self.target_pos[4] = [msg.point.x, msg.point.y]

    def set_pick_request(self, obj_id: int, status: bool)->None:
        req = PickingRequest.Request()
        req.host_name.data = "robot"
        req.obj_id.data = obj_id
        req.status.data = status
        self.picking_request_client.call_async(req) 
    
    def update(self):
        # Ensure all required data is available
        if self.robot_pos is None or self.robot_rot is None or None in self.target_pos or self.battery_level is None:
            return py_trees.common.Status.RUNNING
        
        if self.battery_level < 0.1:
            return py_trees.common.Status.FAILURE

        # Find the nearest target
        nearest_distance = np.inf
        selected_target = None
        for i, target in enumerate(self.target_pos):
            if target is not None:
                distance = np.linalg.norm(np.array(self.robot_pos) - np.array(target))
                # print(f'object {i}: {distance}')
                if distance < nearest_distance and distance <= 0.5:
                    nearest_distance = distance
                    selected_target = i

        if selected_target is None:
            return py_trees.common.Status.FAILURE

        # Picking the nearest target
        self.set_pick_request(selected_target, True)
        if self.verbose: self.logger.info(f"{self.name}: picking object {selected_target} at {self.target_pos[selected_target]}")
        self.blackboard.object_in_hand = selected_target

        # Remove picked target from the robot graph
        for u in self.robot_graph.nodes:
            if euclidean_distance(self.target_pos[selected_target], self.robot_graph.nodes[u]['pos']) < 0.1:
                self.robot_graph.remove_node(u)
                self.blackboard.target_in_robot_graph = self.blackboard.target_in_robot_graph - 1
                break

        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class DropObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None

        self.blackboard = self.attach_blackboard_client(name=name)
        
        try:
            self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.READ)
            self.blackboard.object_in_hand
        except:
            self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.WRITE)
            self.blackboard.object_in_hand = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        # ROS2 Message Setup -----------------------------------------------------------------------------------
        # Subscriber 
        self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)

        # Service Client
        self.picking_request_client = self.node.create_client(PickingRequest, f"/env_{self.env_id}/picking_req")

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data

    def set_pick_request(self, obj_id: int, status: bool)->None:
        req = PickingRequest.Request()
        req.host_name.data = "robot"
        req.obj_id.data = obj_id
        req.status.data = status
        self.picking_request_client.call_async(req) 
    
    def update(self):
        object_in_hand = self.blackboard.object_in_hand
        # Ensure all required data is available
        if self.battery_level is None:
            return py_trees.common.Status.RUNNING
        
        if self.battery_level < 0.1:
            return py_trees.common.Status.FAILURE

        if object_in_hand is None:
            object_in_hand = 4294967295
        
        self.set_pick_request(object_in_hand, False)
        self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.WRITE)
        self.blackboard.object_in_hand = None

        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class Charge(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key="is_charging", access=py_trees.common.Access.WRITE)

        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None

        self.charger_node = 1

        self.robot_pos = None
        self.robot_rot = None

        self.charger_loc = self.robot_graph.nodes[self.charger_node]['pos']
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.battery_subscriber = self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)

        self.charging_request_client = self.node.create_client(ChargingRequest, f"/env_{self.env_id}/robot/charging_req")
        
        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data
    
    def set_charging_request(self, data: Bool)->None:
        req = ChargingRequest.Request()
        req.status.data = data
        self.charging_request_client.call_async(req)
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
        self.robot_rot = [msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w]

    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos is None or self.robot_rot is None or self.battery_level is None:
            return py_trees.common.Status.RUNNING   
        
        if self.battery_level < 0.1 or euclidean_distance(self.charger_loc, self.robot_pos) >= 0.3:
            if self.verbose: self.logger.info('Robot is not at the charger.')
            return py_trees.common.Status.FAILURE
        
        if self.battery_level < 100:
            if self.verbose: self.logger.info(f'Charging -> {self.battery_level}%')
            self.set_charging_request(True)
            self.blackboard.is_charging = True
            return py_trees.common.Status.RUNNING
        else:
            if self.verbose: self.logger.info(f'Finished charging at {self.battery_level}%')
            self.set_charging_request(False)
            self.blackboard.is_charging = False
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")
        self.set_charging_request(False)
        self.blackboard.is_charging = False

# Condition Nodes =========================================================================================================================================

class AreObjectsExistOnInternalMap(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.env_id = env_id
        self.verbose = verbose

        try:
            self.blackboard.register_key(key="target_in_robot_graph", access=py_trees.common.Access.READ)
            self.blackboard.target_in_robot_graph
        except:
            self.blackboard.register_key(key="target_in_robot_graph", access=py_trees.common.Access.WRITE)
            self.blackboard.target_in_robot_graph = False

    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        return True

    def update(self):
        """
        Main behavior logic.
        """
        try:
            if self.blackboard.target_in_robot_graph > 0:
                return py_trees.common.Status.SUCCESS 
            else:
                return py_trees.common.Status.FAILURE   
        except:
            return py_trees.common.Status.FAILURE 
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class IsRobotAtTheCharger(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose

        self.robot_pos = None

        self.robot_graph = robot_graph
        self.ref_node = 1
        self.ref_loc = self.robot_graph.nodes[self.ref_node]['pos']
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
            
    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos == None:
            return py_trees.common.Status.FAILURE   
    
        if euclidean_distance(self.ref_loc, self.robot_pos) < 0.2:
            if self.verbose: self.logger.info('Robot is at the charger')
            return py_trees.common.Status.SUCCESS  
            
        return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class IsRobotAtTheSpawn(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose

        self.robot_pos = None

        self.robot_graph = robot_graph
        self.ref_node = 0
        self.ref_loc = self.robot_graph.nodes[self.ref_node]['pos']
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
            
    def update(self):
        """
        Main behavior logic.
        """
        if self.robot_pos == None:
            return py_trees.common.Status.FAILURE   
    
        if euclidean_distance(self.ref_loc, self.robot_pos) < 0.2:
            if self.verbose: self.logger.info('Robot is at the spawn')
            return py_trees.common.Status.SUCCESS  
            
        return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class IsBatteryOnProperLevel(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)

        try:
            self.blackboard.register_key(key="is_charging", access=py_trees.common.Access.READ)
            self.blackboard.is_charging
        except:
            self.blackboard.register_key(key="is_charging", access=py_trees.common.Access.WRITE)
            self.blackboard.is_charging = False

        self.env_id = env_id
        self.verbose = verbose
        self.battery_level = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.battery_subscriber = self.node.create_subscription(Float32, f"/env_{self.env_id}/robot/battery_level", self.battery_level_callback, 10)

        return True
    
    def battery_level_callback(self, msg):
        self.battery_level = msg.data

    def update(self):
        """
        Main behavior logic.
        """
        if self.battery_level is not None:
            try:
                if self.blackboard.is_charging:
                    if self.battery_level < 100:
                        if self.verbose: self.logger.info('Low Battery')
                        return py_trees.common.Status.FAILURE
                    else:
                        if self.verbose: self.logger.info('Finished Charging')
                        return py_trees.common.Status.SUCCESS
                else:
                    if self.battery_level < 30:
                        if self.verbose: self.logger.info('Low Battery')
                        return py_trees.common.Status.FAILURE
                    else:
                        return py_trees.common.Status.SUCCESS 
            except: 
                if self.verbose: self.logger.info('self.blackboard.is_charging not available')

                if self.battery_level < 30:
                    if self.verbose: self.logger.info('Low Battery')
                    return py_trees.common.Status.FAILURE
                else:
                    return py_trees.common.Status.SUCCESS
        else:
            if self.verbose: self.logger.info('battery level topic not available')
            # return py_trees.common.Status.INVALID
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class AreObjectNearby(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose

        self.robot_pos = None
        self.target_pos = [None, None, None, None, None]
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)

        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target0/tf", self.target_0_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target1/tf", self.target_1_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target2/tf", self.target_2_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target3/tf", self.target_3_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target4/tf", self.target_4_trans_callback, 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
    
    def target_0_trans_callback(self, msg):
        self.target_pos[0] = [msg.point.x, msg.point.y]

    def target_1_trans_callback(self, msg):
        self.target_pos[1] = [msg.point.x, msg.point.y]
    
    def target_2_trans_callback(self, msg):
        self.target_pos[2] = [msg.point.x, msg.point.y]
    
    def target_3_trans_callback(self, msg):
        self.target_pos[3] = [msg.point.x, msg.point.y]
    
    def target_4_trans_callback(self, msg):
        self.target_pos[4] = [msg.point.x, msg.point.y]
            
    def update(self):
        """
        Main behavior logic.
        """
        if None in self.target_pos or self.robot_pos is None:
            return py_trees.common.Status.FAILURE   
    
        for target_pos in self.target_pos:
            if euclidean_distance(target_pos, self.robot_pos) < 0.4:
                if self.verbose: self.logger.info('There exists target nearby.')
                return py_trees.common.Status.SUCCESS  
            
        return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class IsNearbyObjectNotAtGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, goal_node, env_id, verbose = False):
        super().__init__(name)
        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose

        self.robot_pos = None
        self.target_pos = [None, None, None, None, None]

        self.goal_node = goal_node
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)

        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target0/tf", self.target_0_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target1/tf", self.target_1_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target2/tf", self.target_2_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target3/tf", self.target_3_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target4/tf", self.target_4_trans_callback, 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
    
    def target_0_trans_callback(self, msg):
        self.target_pos[0] = [msg.point.x, msg.point.y]

    def target_1_trans_callback(self, msg):
        self.target_pos[1] = [msg.point.x, msg.point.y]
    
    def target_2_trans_callback(self, msg):
        self.target_pos[2] = [msg.point.x, msg.point.y]
    
    def target_3_trans_callback(self, msg):
        self.target_pos[3] = [msg.point.x, msg.point.y]
    
    def target_4_trans_callback(self, msg):
        self.target_pos[4] = [msg.point.x, msg.point.y]
            
    def update(self):
        """
        Main behavior logic.
        """
        if None in self.target_pos or self.robot_pos is None:
            return py_trees.common.Status.FAILURE   
    
        for target_pos in self.target_pos:
            if euclidean_distance(target_pos, self.robot_pos) < 0.4:

                for goal in self.goal_node:
                    if euclidean_distance(target_pos, self.robot_graph.nodes[goal]['pos']) < 0.4:
                        if self.verbose: self.logger.info('The nearby target has reached the goal.')
                        return py_trees.common.Status.FAILURE  
    
                return py_trees.common.Status.SUCCESS  
            
        if self.verbose: self.logger.info('There is no object nearby...')
        return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class IsObjectInHand(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.env_id = env_id
        self.verbose = verbose

        try:
            self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.READ)
            self.blackboard.object_in_hand
        except:
            self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.WRITE)
            self.blackboard.object_in_hand = None

    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        return True

    def update(self):
        """
        Main behavior logic.
        """
        try:
            if self.blackboard.object_in_hand != None:
                return py_trees.common.Status.SUCCESS 
            else:
                return py_trees.common.Status.FAILURE   
        except:
            return py_trees.common.Status.FAILURE 
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class AreXObjectsAtSpawn(py_trees.behaviour.Behaviour):
    def __init__(self, name, robot_graph, env_id, target_number=5, verbose = False):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.robot_graph = robot_graph

        self.env_id = env_id
        self.verbose = verbose

        self.spawn_node = 0

        self.robot_pos = None
        self.target_pos = [None, None, None, None, None]

        self.target_number = target_number

        try:
            self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.READ)
            self.blackboard.object_in_hand
        except:
            self.blackboard.register_key(key="object_in_hand", access=py_trees.common.Access.WRITE)
            self.blackboard.object_in_hand = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)

        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target0/tf", self.target_0_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target1/tf", self.target_1_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target2/tf", self.target_2_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target3/tf", self.target_3_trans_callback, 10)
        self.node.create_subscription(PointStamped, f"/env_{self.env_id}/target4/tf", self.target_4_trans_callback, 10)

        self.obj_at_spawn_pub = self.node.create_publisher(UInt8, f"/env_{self.env_id}/object_at_spawn", 10)
        self.obj_found_pub = self.node.create_publisher(UInt8, f"/env_{self.env_id}/object_found", 10)

        return True
    
    def robot_trans_callback(self, msg):
        self.robot_pos = [msg.transform.translation.x, msg.transform.translation.y]
    
    def target_0_trans_callback(self, msg):
        self.target_pos[0] = [msg.point.x, msg.point.y]

    def target_1_trans_callback(self, msg):
        self.target_pos[1] = [msg.point.x, msg.point.y]
    
    def target_2_trans_callback(self, msg):
        self.target_pos[2] = [msg.point.x, msg.point.y]
    
    def target_3_trans_callback(self, msg):
        self.target_pos[3] = [msg.point.x, msg.point.y]
    
    def target_4_trans_callback(self, msg):
        self.target_pos[4] = [msg.point.x, msg.point.y]
            
    def update(self):
        """
        Main behavior logic.
        """
        if None in self.target_pos or self.robot_pos is None:
            return py_trees.common.Status.FAILURE   

        # Count and publish number of objects at sapwn
        object_in_spawn = 0
        for target_pos in self.target_pos:
            if euclidean_distance(target_pos, self.robot_graph.nodes[self.spawn_node]['pos']) < 0.4:
                object_in_spawn += 1
                
        obj_at_spawn_msg = UInt8()
        obj_at_spawn_msg.data = object_in_spawn
        self.obj_at_spawn_pub.publish(obj_at_spawn_msg)

        # Count and publish number of objects found
        found_obj_in_map = 0
        for i in range(20,30):
            if i in self.robot_graph.nodes:
                found_obj_in_map += 1
        
        if self.blackboard.object_in_hand != None:
            found_obj_in_map += 1

        obj_found_pub_msg = UInt8()
        obj_found_pub_msg.data = found_obj_in_map + object_in_spawn
        self.obj_found_pub.publish(obj_found_pub_msg)

        if object_in_spawn == self.target_number:
            if self.verbose: self.logger.info(f'There exists {self.target_number} objects at the spawn.')
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
    
    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")
# =========================================================================================================================================================

class MoveNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, target, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose

        self.target = target

        self.robot_pos = None
        self.robot_rot = None
        self.current_time = None
        
        self.robot_trans_subscriber = None
        self.cmd_vel_publisher = None

        self.angular_controller = PIDController(K_p = 5.0, K_i = 0.5)
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
            if self.verbose: self.logger.info(f"Successfully setup {self.name}")
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability
        
        self.robot_trans_subscriber = self.node.create_subscription(TransformStamped, f"/env_{self.env_id}/robot/tf", self.robot_trans_callback, 10)
        self.cmd_vel_publisher = self.node.create_publisher(Twist, f"/env_{self.env_id}/robot/cmd_vel", 10)

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
        
    #     if self.verbose: self.logger.info(f'linear error: {distance_to_target} angular error: {angular_error}')

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
    #     if self.verbose: self.logger.info(f"publish linear: {linear_velocity} angular: {angular_velocity}")
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
        
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")