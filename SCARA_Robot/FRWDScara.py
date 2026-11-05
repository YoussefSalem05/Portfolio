import numpy as np
import matplotlib.pyplot as plt

# Robot link lengths in mm
L1 = 45
L2 = 50

# Angles to evaluate (Convert degrees to radians)
theta1 = np.radians([0, 22.5, 45, 67.5, 90])
theta2 = np.radians([0, 22.5, 45, 67.5, 90])

# Using standard Python lists to dynamically store all 25 points
x = []
y = []

# Forward kinematics calculations
print("Calculating End-Effector Positions...")

for t1 in theta1:
    for t2 in theta2:
        # Calculate current position
        curr_x = L1 * np.cos(t1) + L2 * np.cos(t1 + t2)
        curr_y = L1 * np.sin(t1) + L2 * np.sin(t1 + t2)
        
        # Append to our lists
        x.append(curr_x)
        y.append(curr_y)
        
        # Optional: Print each coordinate as it calculates
        # print(f"x = {curr_x:.2f} mm, y = {curr_y:.2f} mm")

# Convert lists back to NumPy arrays if you need to do math on them later
x = np.array(x)
y = np.array(y)

print("\nX Array:\n", np.round(x, 2))
print("\nY Array:\n", np.round(y, 2))

# Plot all the calculated points to visualize the workspace
plt.figure(figsize=(6, 6))
plt.scatter(x, y, color='red', zorder=2)
plt.title("SCARA Robot End-Effector Positions")
plt.xlabel("X Axis (mm)")
plt.ylabel("Y Axis (mm)")
plt.grid(True, linestyle='--', alpha=0.7)
plt.axis('equal') # Ensures 1mm in X looks the same as 1mm in Y
plt.show()