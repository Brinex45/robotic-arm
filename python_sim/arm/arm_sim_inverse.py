import pygame
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import numpy as np
import matplotlib.pyplot as plt
import math as m

d1_test = 37.5
d5_test = 80.5
a2_test = 450
a3_test = 450

unit_tarnsform = 5 #mm
unit_rotation = 1 #degrees

# X = 2
# Y = 0
# Z = 1

X = 0
Y = 0
Z = 0

Z_rot_change = 0 
Y_rot_change = 0

prev_x = 0
prev_y = 0
prev_z = 0

matrix = np.array([
    [0, 0, 1, 531.3],
    [0, 1, 0, 0],
    [-1, 0, 0, 230.5],
    [0, 0, 0, 1]
        ])

def constrain(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Format: RevoluteDH(d, a, alpha, offset=0)
# angles in radians, distances in meters
robot = DHRobot([
    RevoluteDH(d=d1_test, alpha=-np.pi/2),        # Link 1
    RevoluteDH(a=a2_test, alpha=0),             # Link 2
    RevoluteDH(a=a3_test, alpha=0),             # Link 3
    RevoluteDH(alpha=np.pi/2),               # Link 4
    RevoluteDH(d=d5_test, alpha=0)              # Link 5
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
        X_left = joystick.get_axis(0) if abs(joystick.get_axis(0)) > 0.3 else 0
        # print(f"Axis {0}: {X_left:.3f}", end=' | ')

        Y_left = -joystick.get_axis(1) if abs(-joystick.get_axis(1)) > 0.3 else 0
        # print(f"Axis {1}: {Y_left:.3f}", end=' | ')

        X_right = joystick.get_axis(3) if abs(joystick.get_axis(3)) > 0.3 else 0
        # print(f"Axis {3}: {X_right:.3f}", end=' | ')

        L2 = map(joystick.get_axis(2), -1, 1, 0, 1) if abs(map(joystick.get_axis(2), -1, 1, 0, 1)) > 0.3 else 0
        # print(f"Axis {2}: {L2:.3f}", end=' | ')

        R2 = map(joystick.get_axis(5), -1, 1, 0, 1) if abs(map(joystick.get_axis(5), -1, 1, 0, 1)) > 0.3 else 0
        # print(f"Axis {2}: {R2:.3f}", end=' | ')

        L1 = joystick.get_button(4)
        R1 = joystick.get_button(5)

        # print(f"but {4}: {L1:.3f}", end=' | ')
        # print(f"but {5}: {R1:.3f}", end=' | ')
        # print()

        # print()

        # pygame.time.wait(100)  # Add small delay to avoid spamming output

        if X_left != 0: 
            X = X + X_left * unit_tarnsform

        if Y_left != 0:
            Y = Y + Y_left * unit_tarnsform

        if X_right != 0:
            Z = Z + X_right * unit_tarnsform

        if L2 or R2:
            Y_rot_change = (R2 - L2) * unit_rotation
        else:
            Y_rot_change = 0


        if L1 or R1:
            Z_rot_change = (R1 - L1) * unit_rotation
        else:
            Z_rot_change = 0
            
                

        print(X)
        # x = constrain(abs(X) ,0 ,r)
        # y = constrain(abs(Y) ,0 ,r)
        # z = constrain(abs(Z) ,0 ,r)

        # r2 = r**2

        # x_sq = (r2 - Y**2 - (z - 2.15)**2)
        # x_min = -m.sqrt(x_sq) if x_sq >= 0 else 0
        # x_max = m.sqrt(x_sq) if x_sq >= 0 else 0

        # y_sq = (r2 - X**2 - (z - 2.15)**2)
        # y_min = -m.sqrt(y_sq) if y_sq >=0 else 0
        # y_max = m.sqrt(y_sq) if y_sq >=0 else 0

        # z_sq = (r2 - Y**2 - X**2)
        # z_min = -m.sqrt(z_sq) if z_sq >= 0 else 0
        # z_max = m.sqrt(z_sq) if z_sq >= 0 else 0

        # X = constrain(X , x_min ,x_max)
        # Y = constrain(Y , y_min ,y_max)
        # Z = constrain(Z , z_min ,z_max)


        # X = X - 0.01
        # Y = 0
        # Z = 2.15

        # if X_left or X_right or Y_left or Y_rot_change or Z_rot_change:
        Y_rot_rad = m.radians(Y_rot_change)
        Z_rot_rad = m.radians(Z_rot_change)

        My_new = np.array([
            [m.cos(Y_rot_rad), 0, m.sin(Y_rot_rad), 0],
            [0, 1, 0, 0],
            [-m.sin(Y_rot_rad), 0, m.cos(Y_rot_rad), 0],
            [0, 0, 0, 1]
        ])

        Mz_new = np.array([
            [m.cos(Z_rot_rad), -m.sin(Z_rot_rad), 0, 0],
            [m.sin(Z_rot_rad), m.cos(Z_rot_rad) , 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        Tran = np.array([
            [0, 0, 0, X-prev_x],
            [0, 0, 0, Y-prev_y],
            [0, 0, 0, Z-prev_z],
            [0, 0, 0, 0]
        ])
        prev_x = X
        prev_y = Y
        prev_z = Z
        
        # else:
        #     My_new = np.array([
        #         [1, 0, 0, 0],
        #         [0, 1, 0, 0],
        #         [0, 0, 1, 0],
        #         [0, 0, 0, 1]
        #     ])

        #     Mz_new = np.array([
        #         [1, 0, 0, 0],
        #         [0, 1, 0, 0],
        #         [0, 0, 1, 0],
        #         [0, 0, 0, 1]
        #     ])

        #     Tran = np.array([
        #         [1, 0, 0, 0],
        #         [0, 1, 0, 0],
        #         [0, 0, 1, 0],
        #         [0, 0, 0, 1]
        #     ])

        matrix = (matrix @ My_new @ Mz_new) + Tran

        # matrix = np.array([
        #     [-0.7071, 0, 0.7071, X],
        #     [0, 1, 0, Y],
        #     [-0.7071, 0, -0.7071, Z],
        #     [0, 0, 0, 1]
        # ])

        nx1 = matrix[0,0]
        ny1 = matrix[1,0]
        nz1 = matrix[2,0]

        sx1 = matrix[0,1]
        sy1 = matrix[1,1]
        sz1 = matrix[2,1]

        ax1 = matrix[0,2]
        ay1 = matrix[1,2]
        az1 = matrix[2,2]

        Px1 = matrix[0,3]
        Py1 = matrix[1,3]
        Pz1 = matrix[2,3]

        t1 = m.degrees(m.atan2(Py1, Px1))
        t234 = m.degrees(m.atan2(-( (ax1 * m.cos(m.radians(t1))) + (ay1 * m.sin(m.radians(t1)) )),-az1))

        #t5 = (m.atan2((nx1*m.sin(m.radians(t1)) - ny1*m.cos(m.radians(t1))), (sx1*m.sin(m.radians(t1)) - sy1*m.cos(m.radians(t1)))))
        t5 = m.degrees( m.atan2(sz1, -nz1) )
        c = (Px1/m.cos(m.radians(t1))) + d5_test*m.sin(m.radians(t234))
        d = d1_test - d5_test*m.cos(m.radians(t234)) - Pz1

        costheta3 = (c**2 + d**2 - a2_test**2 - a3_test**2) / (2 * a3_test * a2_test)
        costheta3 = min(max(costheta3, -1), 1)
        sintheta3 = m.sqrt(1 - costheta3**2)

        t3 = m.degrees(m.atan2(sintheta3,costheta3))

        r = a3_test*m.cos(m.radians(t3)) + a2_test
        s = a3_test*m.sin(m.radians(t3))

        t2 = m.degrees(m.atan2(r*d - s*c, r*c + s*d))
        t4 = t234 - t3 - t2 + 180

        joint_angles_deg = [t1, t2, t3, t4, t5]
        joint_angles_rad = np.deg2rad(joint_angles_deg)

        # Forward Kinematics
        T_fk = robot.fkine(joint_angles_rad)
        print("🔧 Forward Kinematics Result (End Effector Pose):")
        print(T_fk)

        print(f"Theta 1: {t1:.2f}°")
        print(f"Theta 2: {t2:.2f}°")
        print(f"Theta 3: {t3:.2f}°")
        print(f"Theta 4: {t4:.2f}°")
        print(f"Theta 5: {t5:.2f}°")
        print(f"Theta 234: {t234:.2f}°")

        # Plot the robot
        robot.plot(joint_angles_rad, block=False, backend="pyplot")
        plt.pause(0.001)
        plt.clf()

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()
    plt.close('all')

finally:
    pygame.quit()
    plt.close('all')