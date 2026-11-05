# SCARA Robot: Kinematics & DXF Trajectory Planning

A Python implementation of planar SCARA motion planning, connecting DXF geometry to timed Cartesian waypoints and joint-angle solutions. The project combines forward kinematics, Newton-Raphson inverse kinematics, and tool-path visualization to validate motion before hardware integration.

## System Architecture

The model represents the two revolute joints of a planar SCARA arm. Link lengths and Cartesian coordinates are expressed in millimeters.

```text
DXF geometry
    -> Curve flattening and stroke chaining
    -> Stroke ordering and cutting/rapid motion timing
    -> Newton-Raphson inverse kinematics
    -> Joint-angle CSV/JSON and optional tool-path animation
```

The pipeline uses the previous solved joint angles as the next initial guess and wraps angle changes to maintain continuity. It reports unreachable waypoints and solver failures.

## Kinematic Model

For link lengths $L_1$, $L_2$ and joint angles $\theta_1$, $\theta_2$:

$$x = L_1\cos(\theta_1) + L_2\cos(\theta_1 + \theta_2)$$

$$y = L_1\sin(\theta_1) + L_2\sin(\theta_1 + \theta_2)$$

The inverse solver uses the Jacobian pseudoinverse to iteratively reduce Cartesian position error. The geometric workspace satisfies:

$$|L_1-L_2| \leq \sqrt{x^2+y^2} \leq L_1+L_2$$

## Software Components

| File | Purpose |
| --- | --- |
| `ScaraPipeline.py` | Runs the complete DXF-to-joint-angle workflow. |
| `Trajectory.py` | Parses DXF geometry, chains and orders strokes, builds timed motion profiles, exports waypoints, and animates the tool path. |
| `InvKinematics.py` | Provides the reusable SCARA solver with forward kinematics, Jacobian calculation, reachability checks, and an example. |
| `FRWDScara.py` | Calculates and plots end-effector positions for a sample joint-angle grid. |
| `SCARAROBOTNR.py` | Demonstrates forward/inverse kinematics recovery over a sample configuration grid. |

## Setup

Tested with Python 3.11 on Windows. From this folder, create a dedicated environment:

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install numpy ezdxf matplotlib PySide6
```

On macOS/Linux, activate with `source .venv/bin/activate`. PySide6 provides an interactive plotting backend for Python installations without Tk support.

## Execution Guide

Supply your own DXF file; this folder contains Python source and documentation only. Run commands from this folder, or provide absolute paths.

### Complete pipeline

```powershell
python ScaraPipeline.py "path/to/drawing.dxf" --l1 150 --l2 120 --tool-speed 60 --rapid-speed 200
```

This command sets link lengths to 150 mm and 120 mm, cutting speed to 60 mm/s, and rapid speed to 200 mm/s. The script exports `joint_angles.csv` and `joint_angles.json`, then opens a tool-path animation.

To export without displaying the animation:

```powershell
python ScaraPipeline.py "path/to/drawing.dxf" --l1 150 --l2 120 --tool-speed 60 --rapid-speed 200 --no-animate
```

Use `--out-csv` and `--out-json` to change output paths. Use `python ScaraPipeline.py --help` for curve resolution, animation frame rate, initial joint angles, and solver settings.

### Individual demonstrations

```powershell
python FRWDScara.py
python InvKinematics.py
python SCARAROBOTNR.py
python Trajectory.py "path/to/drawing.dxf" --tool-speed 60 --rapid-speed 200
```

The standalone kinematics examples use link lengths of 45 mm and 50 mm. The complete pipeline accepts link lengths through `--l1` and `--l2`.

## Output and Validation

Each exported joint-angle waypoint includes its index, Cartesian position, cutting/rapid state, timestamp, joint angles in degrees and radians, reachability flag, and IK success flag.

A local validation run using `TopJ2Housing.DXF`, 150/120 mm links, and 60/200 mm/s speeds produced:

- 1 continuous stroke and 80 waypoints.
- 5.40 seconds of planned motion.
- 0 waypoints outside the geometric workspace.
- 0 inverse-kinematics convergence failures.

The DXF and generated results are not included. The animation displays the Cartesian tool path. The model covers planar geometry and numerical IK; physical joint limits, collisions, acceleration constraints, and motor control require additional validation before hardware execution.
