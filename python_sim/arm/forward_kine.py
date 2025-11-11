from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
import matplotlib.pyplot as plt

# Define your robot using standard DH parameters
# Format: RevoluteDH(d, a, alpha, offset=0)
# angles in radians, distances in meters

# Example: 5 DOF robot
robot = DHRobot([
    RevoluteDH(d=37.5, alpha=-np.pi/2),        # Link 1
    RevoluteDH(a=450, alpha=0),             # Link 2
    RevoluteDH(a=450, alpha=0),             # Link 3
    RevoluteDH(alpha=np.pi/2),               # Link 4
    RevoluteDH(d=80.5, alpha=0)              # Link 5
], name='Custom5DOF')

# Set your joint angles here (in radians)
# Example: [0°, -25°, 45°, -90°, 0°]
# joint_angles_deg = [0, -59.82, 111.35, 0, 0]
joint_angles_deg = [0, -90, 65, 90, 0]
joint_angles_rad = np.deg2rad(joint_angles_deg)

# Forward Kinematics
T_fk = robot.fkine(joint_angles_rad)
print("🔧 Forward Kinematics Result (End Effector Pose):")
print(T_fk)

# Plot the robot

robot.plot(joint_angles_rad, block=True)
