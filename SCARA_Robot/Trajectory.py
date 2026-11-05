"""
Robot Tool Trajectory Planner
==============================

Reads a DXF file, converts every entity into a flattened polyline, then:

  1. OPTIMIZES the trajectory:
       - Chains entities that physically touch (shared endpoints) into a
         single continuous stroke, so a line->arc->line that forms one
         connected shape is walked as ONE pass instead of jumping between
         disconnected fragments.
       - Never invents a cutting move between two entities that are not
         actually touching in the drawing (no "additional lines").
       - Handles curves (ARC, CIRCLE, SPLINE, ELLIPSE, LWPOLYLINE with
         bulges, etc.) via ezdxf's `path.flattening()`, which already
         respects curvature.
       - Removes duplicate/near-duplicate points so no point is emitted
         twice within a stroke (closed loops are the one intentional
         exception: last point == first point by definition).
       - Orders the resulting strokes with a nearest-neighbor pass (each
         stroke can be walked forwards or backwards, whichever endpoint is
         closer) to minimize non-cutting travel between disconnected
         pieces. Travel between strokes is explicitly marked as a RAPID
         (pen-up) move, never drawn as if it were part of the geometry.

  2. EXPORTS a clean list of (x, y, pen_state, t) samples suitable for
     feeding into robot kinematics (pen_state distinguishes cutting moves
     from rapid/travel moves so the robot knows when to engage the tool).

  3. Optionally animates the result with matplotlib so you can sanity
     check the tool path before sending it to the robot.

Usage:
    python trajectory_planner.py path.dxf --tool-speed 50 --rapid-speed 150

Dependencies: ezdxf, numpy, matplotlib
"""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np
import ezdxf
from ezdxf import path as ezpath


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class Stroke:
    """A single continuous, already-optimized polyline (one pen-down pass)."""
    points: np.ndarray  # (N, 2) array, no consecutive duplicates
    closed: bool = False
    source_entities: List[str] = field(default_factory=list)

    @property
    def start(self) -> np.ndarray:
        return self.points[0]

    @property
    def end(self) -> np.ndarray:
        return self.points[-1]

    def reversed(self) -> "Stroke":
        return Stroke(self.points[::-1].copy(), self.closed, self.source_entities)


# ---------------------------------------------------------------------------
# Step 1: Read DXF -> one flattened polyline per entity
# ---------------------------------------------------------------------------

def load_entity_paths(dxf_file: str, flatten_distance: float = 0.1) -> List[Stroke]:
    """Convert every supported DXF entity into its own Stroke (not yet
    chained or ordered). Curves are flattened with `flattening()`, which
    walks the true curve geometry (arcs/splines/ellipses), so curvature is
    respected rather than approximated with a naive point sample."""
    doc = ezdxf.readfile(dxf_file)
    msp = doc.modelspace()

    strokes: List[Stroke] = []
    for entity in msp:
        try:
            p = ezpath.make_path(entity)
        except TypeError:
            # Unsupported entity for path conversion (TEXT, DIMENSION, etc.)
            continue

        verts = list(p.flattening(distance=flatten_distance))
        if len(verts) < 2:
            continue

        pts = np.array([(v.x, v.y) for v in verts])
        pts = dedupe_consecutive(pts)
        if len(pts) < 2:
            continue

        closed = np.allclose(pts[0], pts[-1], atol=1e-6)
        strokes.append(Stroke(pts, closed=closed,
                               source_entities=[entity.dxftype()]))
    return strokes


def dedupe_consecutive(pts: np.ndarray, tol: float = 1e-6) -> np.ndarray:
    """Drop consecutive points that are (numerically) the same point, so a
    single physical location is never emitted twice in a row."""
    if len(pts) == 0:
        return pts
    keep = [0]
    for i in range(1, len(pts)):
        if np.linalg.norm(pts[i] - pts[keep[-1]]) > tol:
            keep.append(i)
    return pts[keep]


# ---------------------------------------------------------------------------
# Step 2: Chain strokes that share an endpoint into longer continuous passes
# ---------------------------------------------------------------------------

def chain_strokes(strokes: List[Stroke], tol: float = 1e-3) -> List[Stroke]:
    """Merge strokes whose endpoints coincide (within tol) into a single
    continuous stroke. Only merges at degree-2 junctions (exactly two
    strokes meet at that point) so we never fuse across a branch point,
    which would force revisiting a shared vertex."""

    def key(pt: np.ndarray) -> Tuple[float, float]:
        return (round(pt[0] / tol) * tol, round(pt[1] / tol) * tol)

    # Build endpoint -> list of (stroke_index, 'start'|'end')
    endpoint_map: dict = {}
    for i, s in enumerate(strokes):
        if s.closed:
            continue  # closed loops don't chain with anything
        endpoint_map.setdefault(key(s.start), []).append((i, "start"))
        endpoint_map.setdefault(key(s.end), []).append((i, "end"))

    merged = [False] * len(strokes)
    result: List[Stroke] = []

    for i, s in enumerate(strokes):
        if merged[i] or s.closed:
            continue

        chain_pts = list(s.points)
        chain_sources = list(s.source_entities)
        merged[i] = True
        cur_end = s.end
        cur_start = s.start

        # Extend forward from the current end, then backward from the start
        for direction in ("forward", "backward"):
            growing = True
            while growing:
                growing = False
                probe = cur_end if direction == "forward" else cur_start
                candidates = [c for c in endpoint_map.get(key(probe), [])
                              if not merged[c[0]]]
                if len(candidates) != 1:
                    # 0 candidates: dead end. >1 candidates: a branch point
                    # shared by 3+ strokes -- don't merge, keep as separate
                    # strokes so no vertex gets walked twice.
                    continue
                j, which_end = candidates[0]
                other = strokes[j]
                if other.closed:
                    continue
                other_pts = other.points
                if which_end == "start":
                    add_pts = other_pts[1:]  # drop duplicate shared vertex
                else:
                    add_pts = other_pts[::-1][1:]

                if direction == "forward":
                    chain_pts.extend(add_pts)
                    cur_end = chain_pts[-1]
                else:
                    chain_pts = list(add_pts[::-1]) + chain_pts
                    cur_start = chain_pts[0]

                chain_sources.extend(other.source_entities)
                merged[j] = True
                growing = True

        pts = dedupe_consecutive(np.array(chain_pts))
        closed_now = np.allclose(pts[0], pts[-1], atol=tol)
        result.append(Stroke(pts, closed=closed_now, source_entities=chain_sources))

    # Closed loops pass through untouched
    for i, s in enumerate(strokes):
        if s.closed:
            result.append(s)

    return result


# ---------------------------------------------------------------------------
# Step 3: Order strokes to minimize rapid travel (greedy nearest-neighbor,
#          allowing each stroke to be walked in either direction)
# ---------------------------------------------------------------------------

def order_strokes(strokes: List[Stroke], start_point: Tuple[float, float] = (0.0, 0.0)) -> List[Stroke]:
    remaining = strokes.copy()
    ordered: List[Stroke] = []
    cur_pos = np.array(start_point, dtype=float)

    while remaining:
        best_i, best_flip, best_dist = None, False, None
        for i, s in enumerate(remaining):
            d_start = np.linalg.norm(s.start - cur_pos)
            d_end = np.linalg.norm(s.end - cur_pos)
            if best_dist is None or d_start < best_dist:
                best_dist, best_i, best_flip = d_start, i, False
            if d_end < best_dist:
                best_dist, best_i, best_flip = d_end, i, True

        chosen = remaining.pop(best_i)
        if best_flip:
            chosen = chosen.reversed()
        ordered.append(chosen)
        cur_pos = chosen.end

    return ordered


# ---------------------------------------------------------------------------
# Step 4: Build the final motion profile (constant-velocity cutting moves,
#          separate rapid speed for pen-up travel between strokes)
# ---------------------------------------------------------------------------

def build_motion_profile(strokes: List[Stroke], cut_speed: float,
                          rapid_speed: float, fps: float):
    """Returns dict with per-frame x, y, pen ('cut'/'rapid'), t, plus the
    raw waypoint list (for exporting to robot kinematics without needing
    fixed-frame-rate resampling)."""

    # ---- Build a single list of (point, pen_state) waypoints ----
    waypoints: List[Tuple[np.ndarray, str]] = []
    cur_pos = None
    for s in strokes:
        if cur_pos is not None and np.linalg.norm(s.start - cur_pos) > 1e-6:
            waypoints.append((s.start, "rapid"))  # travel move to next stroke
        for k, pt in enumerate(s.points):
            # first point of a stroke after a rapid is already added as the
            # rapid's destination; mark all stroke points as 'cut'
            waypoints.append((pt, "cut"))
        cur_pos = s.end

    pts = np.array([w[0] for w in waypoints])
    pen = [w[1] for w in waypoints]

    # ---- Timestamp each waypoint using segment-specific speed ----
    t = np.zeros(len(pts))
    for i in range(1, len(pts)):
        dist = np.linalg.norm(pts[i] - pts[i - 1])
        speed = rapid_speed if pen[i] == "rapid" else cut_speed
        speed = max(speed, 1e-6)
        t[i] = t[i - 1] + dist / speed

    total_time = t[-1] if len(t) else 0.0

    # ---- Resample at fixed FPS for animation ----
    total_frames = max(1, int(total_time * fps))
    frame_times = np.linspace(0, total_time, total_frames) if total_time > 0 else np.array([0.0])
    x_anim = np.interp(frame_times, t, pts[:, 0])
    y_anim = np.interp(frame_times, t, pts[:, 1])
    # nearest-neighbor pen state per frame (for coloring the animation)
    pen_idx = np.searchsorted(t, frame_times, side="right") - 1
    pen_idx = np.clip(pen_idx, 0, len(pen) - 1)
    pen_anim = [pen[i] for i in pen_idx]

    return {
        "waypoints": pts,          # (N, 2) raw optimized waypoints
        "pen_states": pen,         # 'cut' / 'rapid' per waypoint
        "waypoint_times": t,       # cumulative time per waypoint
        "total_time": total_time,
        "x_anim": x_anim,
        "y_anim": y_anim,
        "pen_anim": pen_anim,
        "frame_times": frame_times,
    }


# ---------------------------------------------------------------------------
# Step 5: Export for robot kinematics
# ---------------------------------------------------------------------------

def export_waypoints_csv(profile: dict, out_path: str):
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "x", "y", "pen", "t"])
        for i, (pt, pen, t) in enumerate(
            zip(profile["waypoints"], profile["pen_states"], profile["waypoint_times"])
        ):
            writer.writerow([i, f"{pt[0]:.4f}", f"{pt[1]:.4f}", pen, f"{t:.4f}"])


def export_waypoints_json(profile: dict, out_path: str):
    data = [
        {"x": float(pt[0]), "y": float(pt[1]), "pen": pen, "t": float(t)}
        for pt, pen, t in zip(
            profile["waypoints"], profile["pen_states"], profile["waypoint_times"]
        )
    ]
    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------

def plan_trajectory(dxf_file: str, flatten_distance: float = 0.1,
                     chain_tol: float = 1e-3,
                     start_point: Tuple[float, float] = (0.0, 0.0)) -> List[Stroke]:
    raw = load_entity_paths(dxf_file, flatten_distance=flatten_distance)
    if not raw:
        raise ValueError("No valid geometric entities found in DXF.")
    chained = chain_strokes(raw, tol=chain_tol)
    ordered = order_strokes(chained, start_point=start_point)
    return ordered


def animate(strokes: List[Stroke], profile: dict, tool_speed: float, fps: float):
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    all_pts = np.vstack([s.points for s in strokes])

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(all_pts[:, 0], all_pts[:, 1], "k--", alpha=0.3, label="DXF Geometry")
    (cut_line,) = ax.plot([], [], "b-", lw=2, label="Cutting Move")
    (rapid_line,) = ax.plot([], [], "g:", lw=1.5, label="Rapid (pen-up)")
    (tool_head,) = ax.plot([], [], "ro", markersize=8, label="Tool Head")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_title(f"Optimized Tool Trajectory\nSpeed: {tool_speed} mm/s | "
                 f"Duration: {profile['total_time']:.2f}s")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.legend(loc="upper right")
    ax.grid(True)

    x_anim, y_anim, pen_anim = profile["x_anim"], profile["y_anim"], profile["pen_anim"]

    def update(frame):
        xs, ys, pens = x_anim[: frame + 1], y_anim[: frame + 1], pen_anim[: frame + 1]
        cut_x = [x for x, p in zip(xs, pens) if p == "cut"]
        cut_y = [y for y, p in zip(ys, pens) if p == "cut"]
        rap_x = [x for x, p in zip(xs, pens) if p == "rapid"]
        rap_y = [y for y, p in zip(ys, pens) if p == "rapid"]
        cut_line.set_data(cut_x, cut_y)
        rapid_line.set_data(rap_x, rap_y)
        tool_head.set_data([x_anim[frame]], [y_anim[frame]])
        return cut_line, rapid_line, tool_head

    ani = animation.FuncAnimation(
        fig, update, frames=len(x_anim), interval=1000 / fps, blit=True, repeat=True
    )
    plt.show()
    return ani


def main():
    parser = argparse.ArgumentParser(description="DXF -> optimized robot tool trajectory")
    parser.add_argument("dxf_file", help="Path to input DXF file")
    parser.add_argument("--tool-speed", type=float, default=50.0, help="Cutting speed mm/s")
    parser.add_argument("--rapid-speed", type=float, default=150.0, help="Rapid/travel speed mm/s")
    parser.add_argument("--fps", type=float, default=30.0, help="Animation frame rate")
    parser.add_argument("--flatten-distance", type=float, default=0.1,
                         help="Max chordal deviation when flattening curves")
    parser.add_argument("--chain-tol", type=float, default=1e-3,
                         help="Distance tolerance for treating two endpoints as touching")
    parser.add_argument("--out-csv", default="trajectory.csv")
    parser.add_argument("--out-json", default="trajectory.json")
    parser.add_argument("--no-animate", action="store_true")
    args = parser.parse_args()

    strokes = plan_trajectory(args.dxf_file, flatten_distance=args.flatten_distance,
                               chain_tol=args.chain_tol)

    print(f"Optimized into {len(strokes)} continuous stroke(s):")
    for i, s in enumerate(strokes):
        print(f"  Stroke {i}: {len(s.points)} pts, "
              f"{'closed' if s.closed else 'open'}, from {s.source_entities}")

    profile = build_motion_profile(strokes, args.tool_speed, args.rapid_speed, args.fps)
    print(f"\nTotal waypoints: {len(profile['waypoints'])}")
    print(f"Total time: {profile['total_time']:.2f} s")

    export_waypoints_csv(profile, args.out_csv)
    export_waypoints_json(profile, args.out_json)
    print(f"Exported waypoints -> {args.out_csv}, {args.out_json}")

    if not args.no_animate:
        animate(strokes, profile, args.tool_speed, args.fps)


if __name__ == "__main__":
    main()