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
import time
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
        self.is_running = False

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.WRITE)

        # Initialize blackboard variables if it not already initialized
        try:
            self.blackboard.get(f"action_{self.env_id}")
        except:
            self.blackboard.set(f"action_{self.env_id}", '')

    def update(self):
        """
        Main behavior logic.
        """
        self.blackboard.set(f"action_{self.env_id}", self.blackboard.get(f"action_{self.env_id}") + "a")
        self.is_running = True
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
        self.is_running = False

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.WRITE)

        # Initialize blackboard variables if it not already initialized
        try:
            self.blackboard.get(f"action_{self.env_id}")
        except:
            self.blackboard.set(f"action_{self.env_id}", '')

    def update(self):
        """
        Main behavior logic.
        """
        self.blackboard.set(f"action_{self.env_id}", self.blackboard.get(f"action_{self.env_id}") + "b")
        self.is_running = True
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
        self.is_running = False

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.WRITE)

        # Initialize blackboard variables if it not already initialized
        try:
            self.blackboard.get(f"action_{self.env_id}")
        except:
            self.blackboard.set(f"action_{self.env_id}", '')
        
    def update(self):
        """
        Main behavior logic.
        """
        if self.is_running:
            self.is_running = False
            return py_trees.common.Status.SUCCESS

        self.blackboard.set(f"action_{self.env_id}", self.blackboard.get(f"action_{self.env_id}") + "e")
        self.is_running = True
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
        self.is_running = False

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.WRITE)

        # Initialize blackboard variables if it not already initialized
        try:
            self.blackboard.get(f"action_{self.env_id}")
        except:
            self.blackboard.set(f"action_{self.env_id}", '')

    def update(self):
        """
        Main behavior logic.
        """
        if self.is_running:
            self.is_running = False
            return py_trees.common.Status.SUCCESS

        self.blackboard.set(f"action_{self.env_id}", self.blackboard.get(f"action_{self.env_id}") + "c")
        self.is_running = True
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
        self.is_running = False

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.WRITE)

        # Initialize blackboard variables if it not already initialized
        try:
            self.blackboard.get(f"action_{self.env_id}")
        except:
            self.blackboard.set(f"action_{self.env_id}", '')

    def update(self):
        """
        Main behavior logic.
        """
        if self.is_running:
            self.is_running = False
            return py_trees.common.Status.SUCCESS

        self.blackboard.set(f"action_{self.env_id}", self.blackboard.get(f"action_{self.env_id}") + "f")
        self.is_running = True
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
        self.is_running = False

        self.blackboard = self.attach_blackboard_client(name=name)

        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key=f"action_{self.env_id}", access=py_trees.common.Access.WRITE)

        # Initialize blackboard variables if it not already initialized
        try:
            self.blackboard.get(f"action_{self.env_id}")
        except:
            self.blackboard.set(f"action_{self.env_id}", '')
        
    def update(self):
        """
        Main behavior logic.
        """
        if self.is_running:
            self.is_running = False
            return py_trees.common.Status.SUCCESS

        self.blackboard.set(f"action_{self.env_id}", self.blackboard.get(f"action_{self.env_id}") + "g")
        self.is_running = True
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
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=f"env_state_{self.env_id}", access=py_trees.common.Access.READ)

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['C', 'D', 'G']

        try:
            if self.blackboard.get(f"env_state_{self.env_id}") in accepted_states:
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
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=f"env_state_{self.env_id}", access=py_trees.common.Access.READ)

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['A', 'F', 'G']

        try:
            if self.blackboard.get(f"env_state_{self.env_id}") in accepted_states:
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
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=f"env_state_{self.env_id}", access=py_trees.common.Access.READ)

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['D']

        try:
            if self.blackboard.get(f"env_state_{self.env_id}") in accepted_states:
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
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=f"env_state_{self.env_id}", access=py_trees.common.Access.READ)

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['E', 'F']

        try:
            if self.blackboard.get(f"env_state_{self.env_id}") in accepted_states:
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
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=f"env_state_{self.env_id}", access=py_trees.common.Access.READ)

    def update(self):
        """
        Main behavior logic.
        """
        accepted_states = ['H']

        try:
            if self.blackboard.get(f"env_state_{self.env_id}") in accepted_states:
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