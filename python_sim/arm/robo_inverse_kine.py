import roboticstoolbox as rtb
from spatialmath import SE3
from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np

# Define 5-DOF custom robot
robot = DHRobot([
    RevoluteDH(d=2.15, alpha=np.pi/2),        # Link 1
    RevoluteDH(a=1.778, alpha=0),             # Link 2
    RevoluteDH(a=1.778, alpha=0),             # Link 3
    RevoluteDH(alpha=-np.pi/2),               # Link 4
    RevoluteDH(d=1.295, alpha=0)              # Link 5
], name='Custom5DOF')

print(robot)

# Define joint angles in degrees
joint_angles_deg = [0, 0, 0, -90, 0]
joint_angles_rad = np.deg2rad(joint_angles_deg)

# Forward Kinematics
T_fk = robot.fkine(joint_angles_rad)
print("🔧 Forward Kinematics Result (End Effector Pose):")
print(T_fk)

# Desired pose matrix
T_array = np.array([
    [-1, 0, 0, 2],
    [0, 1, 0, 0],
    [0, 0, -1, 1],
    [0, 0, 0, 1]
])
T_desired = SE3(T_array)

# Inverse Kinematics
solution = robot.ikine_LM(T_desired)

if solution.success:
    q_sol_rad = solution.q
    q_sol_deg = np.rad2deg(q_sol_rad)

    print("\n IK Solution Found! Joint Angles (degrees):")
    for i, angle in enumerate(q_sol_deg):
        print(f"Joint {i+1}: {angle:.2f}°")

    # Forward check
    T_check = robot.fkine(q_sol_rad)
    error = np.linalg.norm(T_check.A - T_desired.A)
    print("\n Forward Kinematics Check:")
    print(T_check)
    print(f"\n Pose Error: {error:.6e}")
    if error > 1e-3:
        print(" Pose mismatch. Consider reviewing link parameters.")
    else:
        print(" Pose verified successfully!")

    # Plot result
    robot.plot(q_sol_rad, block=True)

    # Optional: Only use teach if matplotlib is compatible
    try:
        import matplotlib
        major = int(matplotlib.__version__.split(".")[0])
        minor = int(matplotlib.__version__.split(".")[1])
        if major <= 3 and minor <= 4:
            robot.teach(q_sol_rad)
        else:
            print("Skipping teach panel due to matplotlib version mismatch.")
    except Exception as e:
        print(f" Could not launch teach panel: {e}")

else:
    print(" No IK solution found for the desired pose.")
