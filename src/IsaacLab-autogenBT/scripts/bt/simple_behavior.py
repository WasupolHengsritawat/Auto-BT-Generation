import py_trees
import py_trees_ros
import roboticstoolbox as rtb
from spatialmath import SE3
from spatialmath.base import q2r
from geometry_msgs.msg import Twist, PointStamped, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float32, Bool, UInt8, String
from tf_transformations import euler_from_quaternion
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import networkx as nx
import random
import math
import sys 

from autogen_bt_interface.msg import StringStamped

# Behavior Nodes ==========================================================================================================================================

class PatrolNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_publisher = self.node.create_publisher(StringStamped, f"/env_{self.env_id}/robot/action", 10)
        
        return True

    def update(self):
        """
        Main behavior logic.
        """
        msg = StringStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = 'a'
        self.action_publisher.publish(msg)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class FindTargetNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_publisher = self.node.create_publisher(StringStamped, f"/env_{self.env_id}/robot/action", 10)
        
        return True

    def update(self):
        """
        Main behavior logic.
        """
        msg = StringStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = 'b'
        self.action_publisher.publish(msg)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class GoToSpawnNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_publisher = self.node.create_publisher(StringStamped, f"/env_{self.env_id}/robot/action", 10)
        
        return True

    def update(self):
        """
        Main behavior logic.
        """
        msg = StringStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = 'e'
        self.action_publisher.publish(msg)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class GoToNearestTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_publisher = self.node.create_publisher(StringStamped, f"/env_{self.env_id}/robot/action", 10)
        
        return True

    def update(self):
        """
        Main behavior logic.
        """
        msg = StringStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = 'c'
        self.action_publisher.publish(msg)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_publisher = self.node.create_publisher(StringStamped, f"/env_{self.env_id}/robot/action", 10)
        
        return True

    def update(self):
        """
        Main behavior logic.
        """
        msg = StringStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = 'f'
        self.action_publisher.publish(msg)

        return py_trees.common.Status.RUNNING 

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
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.action_publisher = self.node.create_publisher(StringStamped, f"/env_{self.env_id}/robot/action", 10)
        
        return True

    def update(self):
        """
        Main behavior logic.
        """
        msg = StringStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.data = 'g'
        self.action_publisher.publish(msg)

        return py_trees.common.Status.RUNNING 

    def terminate(self, new_status):
        """
        Stop the robot on termination.
        """
        if self.verbose: self.logger.debug(f"{self.name}: terminate({new_status})")

# Condition Nodes =========================================================================================================================================

class AreObjectsExistOnInternalMap(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        self.state = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.state_subscriber = self.node.create_subscription(String, f"/env_{self.env_id}/robot/state", self.state_callback, 10)

        return True
    
    def state_callback(self, msg):
        self.state = msg.data

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['C', 'D', 'G']

        try:
            if self.state in accepted_states:
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

class IsRobotAtTheSpawn(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        self.state = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.state_subscriber = self.node.create_subscription(String, f"/env_{self.env_id}/robot/state", self.state_callback, 10)

        return True
    
    def state_callback(self, msg):
        self.state = msg.data

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['A', 'F', 'G']

        try:
            if self.state in accepted_states:
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

class AreObjectNearby(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        self.state = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.state_subscriber = self.node.create_subscription(String, f"/env_{self.env_id}/robot/state", self.state_callback, 10)

        return True
    
    def state_callback(self, msg):
        self.state = msg.data

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['D']

        try:
            if self.state in accepted_states:
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

class IsObjectInHand(py_trees.behaviour.Behaviour):
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        self.state = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.state_subscriber = self.node.create_subscription(String, f"/env_{self.env_id}/robot/state", self.state_callback, 10)

        return True
    
    def state_callback(self, msg):
        self.state = msg.data

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['E', 'F']

        try:
            if self.state in accepted_states:
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
    def __init__(self, name, env_id, verbose = False):
        super().__init__(name)
        self.env_id = env_id
        self.verbose = verbose
        self.state = None
        
    def setup(self, **kwargs):
        """
        One-time setup to initialize ROS2 publishers and subscribers.
        """
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(self.qualified_name)
            raise KeyError(error_message) from e  # 'direct cause' traceability

        self.state_subscriber = self.node.create_subscription(String, f"/env_{self.env_id}/robot/state", self.state_callback, 10)

        return True
    
    def state_callback(self, msg):
        self.state = msg.data

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['H']

        try:
            if self.state in accepted_states:
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