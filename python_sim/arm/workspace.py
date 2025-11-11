import pygame
from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
from spatialmath import SE3
import matplotlib.pyplot as plt

# Define pi constant
pi = np.pi

# Create robot with custom DH parameters and joint limits
robot = DHRobot([
    RevoluteDH(d=2.15, alpha=-pi/2, qlim=[-pi, pi]),      # Link 1
    RevoluteDH(a=1.778, alpha=0, qlim=[-pi/2, pi/2]),     # Link 2
    RevoluteDH(a=1.778, alpha=0, qlim=[-pi/2, pi/2]),     # Link 3
    RevoluteDH(alpha=pi/2, qlim=[-pi, pi]),               # Link 4
    RevoluteDH(d=1.295, alpha=0, qlim=[-pi, pi])          # Link 5
], name='Custom5DOF')

# Reference config for plotting
q_ref = [0, 0, 0, -pi/2, 0]
# robot.plot(q_ref, block=False)

# Generate random points in workspace
try:
    points = []
    for _ in range(5000):
        q_rand = np.array([
            np.random.uniform(j.qlim[0], j.qlim[1]) for j in robot.links
        ])
        T = robot.fkine(q_rand)
        points.append(T.t)

    points = np.array(points)

    # Get min/max ranges
    x_min, y_min, z_min = np.min(points, axis=0)
    x_max, y_max, z_max = np.max(points, axis=0)

    print(f"X Range: {x_min:.2f} to {x_max:.2f}")
    print(f"Y Range: {y_min:.2f} to {y_max:.2f}")
    print(f"Z Range: {z_min:.2f} to {z_max:.2f}")

    # Plot the workspace
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1)
    ax.set_title("Workspace (random sampling)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.show(block=True)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    plt.close('all')
