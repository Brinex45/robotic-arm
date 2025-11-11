import pygame
from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np
import matplotlib.pyplot as plt

theta_1 = 0
theta_2 = 0
theta_3 = 0
theta_4 = -90
theta_5 = 0

prev_X_left = 0
prev_Y_left = 0
prev_X_right = 0

# Format: RevoluteDH(d, a, alpha, offset=0)
# angles in radians, distances in meters
robot = DHRobot([
    RevoluteDH(d=2.15, alpha=np.pi/2),        # Link 1
    RevoluteDH(a=1.778, alpha=0),             # Link 2
    RevoluteDH(a=1.778, alpha=0),             # Link 3
    RevoluteDH(alpha=-np.pi/2),               # Link 4
    RevoluteDH(d=1.295, alpha=0)              # Link 5
], name='Custom5DOF')

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()

# Detect joystick
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Make sure your PS4 controller is connected.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Connected to: {joystick.get_name()}")
print(f"Number of Axes: {joystick.get_numaxes()}")
print(f"Number of Buttons: {joystick.get_numbuttons()}")
print(f"Number of Hats (D-pad): {joystick.get_numhats()}")

print("\n--- Listening for controller input ---\n(Press Ctrl+C to quit)")

try:
    while True:
        pygame.event.pump()  # Refresh event queue

        # Axes (analog sticks and triggers)
        X_left = joystick.get_axis(0) if abs(joystick.get_axis(0)) > 0.15 else 0
        # print(f"Axis {0}: {X_left:.3f}", end=' | ')

        Y_left = -joystick.get_axis(1) if abs(-joystick.get_axis(1)) > 0.15 else 0
        # print(f"Axis {1}: {Y_left:.3f}", end=' | ')

        X_right = joystick.get_axis(3) if abs(joystick.get_axis(3)) > 0.15 else 0
        # print(f"Axis {3}: {X_right:.3f}", end=' | ')
        # print()

        # pygame.time.wait(100)  # Add small delay to avoid spamming output

        if X_left or abs(X_left - prev_X_left): 
            theta_1 = theta_1 + X_left * 5
            prev_X_left = X_left

        if Y_left or Y_left - prev_Y_left:
            theta_4 = theta_4 + Y_left * 5
            prev_Y_left = Y_left

        if X_right or X_right - prev_X_right:
            theta_5 = theta_5 + X_right * 5
            prev_X_right = X_right

        # joint angles here (in radians)
        joint_angles_deg = [theta_1, theta_2, theta_3, theta_4, theta_5]
        joint_angles_rad = np.deg2rad(joint_angles_deg)

        # Forward Kinematics
        T_fk = robot.fkine(joint_angles_rad)
        print("🔧 Forward Kinematics Result (End Effector Pose):")
        print(T_fk)

        # Plot the robot
        robot.plot(joint_angles_rad, block=False, backend="pyplot")
        plt.pause(0.001)
        plt.clf() 

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()

finally:
    pygame.quit()
    plt.close('all')