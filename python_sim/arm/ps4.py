import pygame


def constrain(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

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

        # # Axes (analog sticks and triggers)
        # for i in range(joystick.get_numaxes()):
        #     axis_val = joystick.get_axis(i)
        #     print(f"Axis {i}: {axis_val:.3f}", end=' | ')
        # print()

        # # Buttons (X, Circle, Triangle, Square, L1, R1, etc.)
        # for i in range(joystick.get_numbuttons()):
        #     button = joystick.get_button(i)
        #     if button:
        #         print(f"Button {i} pressed")

        # # D-pad (hat)
        # for i in range(joystick.get_numhats()):
        #     hat = joystick.get_hat(i)
        #     if hat != (0, 0):
        #         print(f"D-pad Hat {i}: {hat}")

        # pygame.time.wait(100)  # Add small delay to avoid spamming output

        X_left = joystick.get_axis(0) if abs(joystick.get_axis(0)) > 0.3 else 0
        print(f"Axis {0}: {X_left:.3f}", end=' | ')

        Y_left = -joystick.get_axis(1) if abs(-joystick.get_axis(1)) > 0.3 else 0
        print(f"Axis {1}: {Y_left:.3f}", end=' | ')

        X_right = joystick.get_axis(3) if abs(joystick.get_axis(3)) > 0.3 else 0
        print(f"Axis {3}: {X_right:.3f}", end=' | ')

        L2 = map(joystick.get_axis(2), -1, 1, 0, 1) if abs(map(joystick.get_axis(2), -1, 1, 0, 1)) > 0.3 else 0
        print(f"Axis {2}: {L2:.3f}", end=' | ')

        R2 = map(joystick.get_axis(5), -1, 1, 0, 1) if abs(map(joystick.get_axis(5), -1, 1, 0, 1)) > 0.3 else 0
        print(f"Axis {2}: {R2:.3f}", end=' | ')

        L1 = joystick.get_button(4)
        R1 = joystick.get_button(5)

        print(f"but {4}: {L1:.3f}", end=' | ')
        print(f"but {5}: {R1:.3f}", end=' | ')
        print()

except KeyboardInterrupt:
    print("\nExiting...")
    pygame.quit()
