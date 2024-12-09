import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt

# Link lengths and offsets
L1, L2, L3, L4, L5, L6, L7, L8 = 0.1, 0.284, 0.0892, 0.4250, 0.3920, 0.1093, 0.09475, 0.19085

# Define the robot using MDH parameters
robot = rtb.DHRobot([
    rtb.RevoluteMDH(a =  L1, alpha =        0, d = L2+L3, qlim = np.array([  -np.pi, np.pi])),   # shoulder_pan_joint
    rtb.RevoluteMDH(a =   0, alpha =  np.pi/2, d =     0, qlim = np.array([-2*np.pi,      0])),   # shoulder_lift_joint
    rtb.RevoluteMDH(a = -L4, alpha =        0, d =     0, qlim = np.array([  -np.pi,  np.pi])),   # elbow_joint
    rtb.RevoluteMDH(a = -L5, alpha =        0, d =    L6, qlim = np.array([  -np.pi,  np.pi])),   # wrist_1_joint
    rtb.RevoluteMDH(a =   0, alpha = -np.pi/2, d =   -L7, qlim = np.array([  -np.pi,  np.pi])),   # wrist_2_joint
    rtb.RevoluteMDH(a =   0, alpha =  np.pi/2, d =     0, qlim = np.array([  -np.pi,  np.pi]))    # wrist_3_joint
], name="Jackal_UR5")
print(robot)

# Add the tool transformation
robot.tool = np.array([[0,1,0,0],[0,0,-1,0],[1,0,0,L8],[0,0,0,1]])

fig = robot.teach(q=[0.0]*6)