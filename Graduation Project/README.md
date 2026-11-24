# Graduation Project

## AR4 Welding MPC

My graduation project explores robotic welding with a six-axis AR4 robot. The simulation takes a Cartesian welding path, uses inverse kinematics to initialize a joint trajectory, and optimizes the motion with nonlinear model predictive control. ROS 2 Humble and RViz provide visualization and motion replay.

For the demonstration, I used a CSV with 21 positions and orientations along a 30 cm seam. The AR4 URDF supplies the geometry. The controller looks 1.2 seconds ahead, checks the proposed motion against joint limits, and applies the first velocity command. RViz shows the requested path, the current prediction, and the motion the simulated arm actually executed.

All 110 commands were applied in the ideal simulation. Across 111 sampled poses, the largest position error was **0.0342 mm**. Physical welding accuracy requires separate calibration and hardware tests.

- [Pipeline diagrams, ROS design and mathematical formulation](https://github.com/YoussefSalem05/AR4-Welding-MPC/blob/main/README.md)
- [Experiment runbook for developers and AI agents](https://github.com/YoussefSalem05/AR4-Welding-MPC/blob/main/docs/README-AGENTS.md)

[AR4 Welding MPC on GitHub](https://github.com/YoussefSalem05/AR4-Welding-MPC) is a private repository. Access is required to view the source.
