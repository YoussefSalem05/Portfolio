"""
Full SCARA Pipeline: DXF -> Optimized Trajectory -> Inverse Kinematics -> Joint Angles
========================================================================================

Chains Trajectory.py (DXF parsing + trajectory optimization) with
InvKinematics.py (Newton-Raphson SCARA IK) into a single run.

For every optimized waypoint (both cutting AND rapid moves need a joint
solution, since the robot has to physically move through both), IK is
solved with the PREVIOUS point's solved angles as the initial guess
("warm start"). This does two important things:
  1. Massively speeds up Newton-Raphson convergence, since consecutive
     trajectory points are close together (the previous solution is
     already a great starting guess).
  2. Keeps the arm on the SAME elbow-up/elbow-down branch throughout a
     continuous stroke, avoiding sudden joint-space jumps/flips that a
     fresh [0, 0] guess could produce at some points.

Points outside the physical reach of the arm (r < |L1-L2| or r > L1+L2)
are detected up front and reported, rather than silently handed to
Newton-Raphson (which would just fail to converge or converge to the
nearest reachable point, distorting the path).

Usage:
    python -m pip install ezdxf numpy matplotlib
    python ScaraPipeline.py TopJ2Housing.DXF --l1 45 --l2 50 --tool-speed 50

Run from the project directory, or supply full paths to the script and DXF.

Outputs a CSV/JSON with, per waypoint: index, x, y, pen, t, theta1_deg,
theta2_deg, theta1_rad, theta2_rad, ik_success.
"""

from __future__ import annotations

import argparse
import csv
import json

import numpy as np

from Trajectory import plan_trajectory, build_motion_profile
from InvKinematics import ScaraNewtonRaphson


def solve_ik_for_path(profile: dict, robot: ScaraNewtonRaphson,
                       initial_guess=(0.0, 0.0), tol=1e-4, max_iter=100):
    """Warm-started IK sweep over every waypoint in the optimized
    trajectory. Returns a list of per-waypoint result dicts."""

    waypoints = profile["waypoints"]
    pen_states = profile["pen_states"]
    times = profile["waypoint_times"]

    results = []
    theta_guess = np.array(initial_guess, dtype=float)
    unreachable_count = 0
    failed_count = 0

    for i, (pt, pen, t) in enumerate(zip(waypoints, pen_states, times)):
        reachable = robot.is_reachable(pt)
        if not reachable:
            unreachable_count += 1

        theta_raw, success = robot.inverse_kinematics(
            pt, theta_guess, tol=tol, max_iter=max_iter
        )

        # Newton-Raphson has no notion of "shortest path" in angle space:
        # near a singularity it can wind through several extra full turns
        # (e.g. -307354 deg) while still satisfying the position tolerance,
        # since cos/sin are periodic. That's mechanically equivalent but
        # useless to send to a motor. Wrap the result to the representative
        # angle closest to the previous guess so joint values stay
        # continuous and bounded, instead of winding up.
        delta = theta_raw - theta_guess
        wrapped_delta = np.arctan2(np.sin(delta), np.cos(delta))
        theta = theta_guess + wrapped_delta

        if not success:
            failed_count += 1
            # Don't let a divergent result poison the warm start for the
            # next point -- keep the last good guess.
        else:
            theta_guess = theta

        results.append({
            "index": i,
            "x": float(pt[0]),
            "y": float(pt[1]),
            "pen": pen,
            "t": float(t),
            "theta1_rad": float(theta[0]),
            "theta2_rad": float(theta[1]),
            "theta1_deg": float(np.degrees(theta[0])),
            "theta2_deg": float(np.degrees(theta[1])),
            "reachable": bool(reachable),
            "ik_success": bool(success),
        })

    print(f"IK sweep complete: {len(results)} waypoints, "
          f"{unreachable_count} outside workspace, "
          f"{failed_count} failed to converge.")
    return results


def export_csv(results, out_path: str):
    fieldnames = ["index", "x", "y", "pen", "t", "theta1_deg", "theta2_deg",
                  "theta1_rad", "theta2_rad", "reachable", "ik_success"]
    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in results:
            writer.writerow(r)


def export_json(results, out_path: str):
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)


def main():
    parser = argparse.ArgumentParser(
        description="DXF -> optimized trajectory -> SCARA inverse kinematics -> joint angles")
    parser.add_argument("dxf_file", help="Path to input DXF file")

    # Trajectory planning args
    parser.add_argument("--tool-speed", type=float, default=50.0, help="Cutting speed mm/s")
    parser.add_argument("--rapid-speed", type=float, default=150.0, help="Rapid/travel speed mm/s")
    parser.add_argument("--fps", type=float, default=30.0, help="Animation frame rate")
    parser.add_argument("--flatten-distance", type=float, default=0.1,
                         help="Max chordal deviation when flattening curves")
    parser.add_argument("--chain-tol", type=float, default=1e-3,
                         help="Distance tolerance for treating two endpoints as touching")

    # IK / robot args
    parser.add_argument("--l1", type=float, default=45.0, help="SCARA link 1 length (mm)")
    parser.add_argument("--l2", type=float, default=50.0, help="SCARA link 2 length (mm)")
    parser.add_argument("--theta1-init", type=float, default=0.0,
                         help="Initial theta1 guess in degrees for the very first waypoint")
    parser.add_argument("--theta2-init", type=float, default=0.0,
                         help="Initial theta2 guess in degrees for the very first waypoint")
    parser.add_argument("--ik-tol", type=float, default=1e-4, help="IK position error tolerance (mm)")
    parser.add_argument("--ik-max-iter", type=int, default=100, help="Max Newton-Raphson iterations")

    # Output args
    parser.add_argument("--out-csv", default="joint_angles.csv")
    parser.add_argument("--out-json", default="joint_angles.json")
    parser.add_argument("--no-animate", action="store_true")
    args = parser.parse_args()

    # ---- Stage 1: DXF -> optimized trajectory ----
    strokes = plan_trajectory(args.dxf_file, flatten_distance=args.flatten_distance,
                               chain_tol=args.chain_tol)
    print(f"Optimized into {len(strokes)} continuous stroke(s).")

    profile = build_motion_profile(strokes, args.tool_speed, args.rapid_speed, args.fps)
    print(f"Total waypoints: {len(profile['waypoints'])}  |  "
          f"Total time: {profile['total_time']:.2f}s")

    # ---- Stage 2: check workspace fit before solving ----
    robot = ScaraNewtonRaphson(L1=args.l1, L2=args.l2)
    r_min, r_max = abs(args.l1 - args.l2), args.l1 + args.l2
    dists = np.linalg.norm(profile["waypoints"], axis=1)
    print(f"Robot reach: {r_min:.1f}mm - {r_max:.1f}mm  |  "
          f"Path bounds from origin: {dists.min():.1f}mm - {dists.max():.1f}mm")
    if dists.max() > r_max or dists.min() < r_min:
        print("WARNING: some waypoints fall outside the robot's workspace. "
              "Check your DXF origin/scale relative to the SCARA base, or "
              "adjust --l1/--l2.")

    # ---- Stage 3: DXF/trajectory frame -> IK -> joint angles ----
    initial_guess = (np.radians(args.theta1_init), np.radians(args.theta2_init))
    results = solve_ik_for_path(profile, robot, initial_guess=initial_guess,
                                 tol=args.ik_tol, max_iter=args.ik_max_iter)

    export_csv(results, args.out_csv)
    export_json(results, args.out_json)
    print(f"Exported joint angles -> {args.out_csv}, {args.out_json}")

    if not args.no_animate:
        from Trajectory import animate
        animate(strokes, profile, args.tool_speed, args.fps)


if __name__ == "__main__":
    main()
