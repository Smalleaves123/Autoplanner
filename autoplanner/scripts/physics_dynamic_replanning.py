#!/usr/bin/env python3
"""Closed-loop D* Lite replanning with a PyBullet racecar.

The run uses the real occupancy grid for both planning and static collision
geometry. A free cell on the active path is occupied during execution, the
same obstacle is inserted into PyBullet, D* Lite replans from the current
cell, and the new path is converted into a fresh tracking trajectory.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Any

import autoplanner
import autompc

from physics_backend_smoke import PhysicsOptions, PyBulletRacecarSimulator
from physics_tracking_benchmark import load_obstacle_rectangles


def closest_index(path: list[Any], x: float, y: float) -> int:
    return min(range(len(path)), key=lambda i: (path[i].x - x) ** 2
               + (path[i].y - y) ** 2)


def closest_trajectory(trajectory: list[Any], x: float, y: float) -> tuple[int, Any]:
    return min(
        enumerate(trajectory),
        key=lambda item: (item[1].x - x) ** 2 + (item[1].y - y) ** 2)


def make_trajectory(path: list[Any], args: argparse.Namespace) -> list[Any]:
    options = autompc.TrajectoryOptions()
    options.sample_spacing = args.sample_spacing
    options.target_velocity = args.velocity
    options.max_velocity = args.max_velocity
    options.max_acceleration = args.max_acceleration
    options.max_deceleration = args.max_deceleration
    options.max_lateral_acceleration = args.max_lateral_acceleration
    return autompc.generate_trajectory(
        [autompc.Waypoint2d(point.x, point.y) for point in path], options)


def inflate_cell(grid: Any, cell_x: int, cell_y: int,
                 radius: float) -> None:
    for y in range(grid.height()):
        for x in range(grid.width()):
            dx = max(abs(x - cell_x) - 0.5, 0.0)
            dy = max(abs(y - cell_y) - 0.5, 0.0)
            if math.hypot(dx, dy) <= radius:
                grid.set_occupied(x, y, True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--output-dir",
                        default="autoplanner/results/physics_dynamic")
    parser.add_argument("--controller", choices=("stanley", "mpc"),
                        default="mpc")
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--steps-per-frame", type=int, default=80)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--wheelbase", type=float, default=0.325)
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-steering", type=float, default=0.7)
    parser.add_argument("--max-acceleration", type=float, default=1.5)
    parser.add_argument("--max-deceleration", type=float, default=2.0)
    parser.add_argument("--max-steering-rate", type=float, default=1.5)
    parser.add_argument("--mpc-horizon", type=int, default=15)
    parser.add_argument("--sample-spacing", type=float, default=0.5)
    parser.add_argument("--max-lateral-acceleration", type=float, default=1.5)
    parser.add_argument("--planner-robot-radius", type=float, default=0.4)
    parser.add_argument("--obstacle-ahead", type=int, default=12)
    args = parser.parse_args()
    if args.frames <= 0 or args.steps_per_frame <= 0:
        parser.error("frames and steps-per-frame must be positive")

    root = Path(__file__).resolve().parents[2]
    map_path = (root / args.map).resolve()
    output_dir = (root / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    grid = autoplanner.GridMap()
    if not grid.load_from_txt(str(map_path)):
        raise RuntimeError(f"failed to load map: {map_path}")
    grid.inflate_obstacles(args.planner_robot_radius)
    start = autoplanner.Point2i(1, 1)
    goal = autoplanner.Point2i(grid.width() - 2, grid.height() - 2)
    dstar = autoplanner.DStarLitePlanner(True)
    result = dstar.plan(grid, start, goal)
    if not result.success:
        raise RuntimeError(f"initial planning failed: {result.message}")

    current_path = list(result.path)
    trajectory = make_trajectory(current_path, args)
    if not trajectory:
        raise RuntimeError("initial trajectory is empty")
    physics_options = PhysicsOptions(
        dt=args.dt, wheelbase=args.wheelbase,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        max_deceleration=args.max_deceleration,
        max_steering=args.max_steering,
        max_steering_rate=args.max_steering_rate,
        obstacle_rectangles=load_obstacle_rectangles(map_path),
    )
    simulator = PyBulletRacecarSimulator(physics_options)
    simulator.reset(trajectory[0].x, trajectory[0].y + 0.5,
                    trajectory[0].theta, 0.0)
    if args.controller == "stanley":
        controller = autompc.StanleyController(0.5, args.wheelbase)
    else:
        controller = autompc.MPCController(
            args.mpc_horizon, args.dt, args.wheelbase,
            args.max_velocity, args.max_steering,
            args.max_acceleration, args.max_deceleration,
            args.max_steering_rate)

    csv_path = output_dir / f"{args.controller}_racecar_dynamic.csv"
    summary_path = output_dir / f"{args.controller}_racecar_dynamic.json"
    fields = [
        "frame", "step", "x", "y", "theta", "v", "ref_x", "ref_y",
        "ref_theta", "command_velocity", "command_steering", "replanned",
        "dynamic_obstacle_x", "dynamic_obstacle_y", "dstar_time_ms",
        "cross_track", "goal_distance", "obstacle_contacts", "safe_stop",
    ]
    dynamic_obstacle: tuple[int, int] | None = None
    replanning_count = 0
    replanning_time_ms = 0.0
    collision_steps = 0
    safe_stop = False
    goal_reached = False
    rows: list[dict[str, float | int]] = []
    try:
        for frame in range(args.frames):
            replanned = False
            dstar_time_ms = 0.0
            if frame == 1 and len(current_path) > args.obstacle_ahead + 2:
                obstacle_index = min(args.obstacle_ahead, len(current_path) - 2)
                candidate = current_path[obstacle_index]
                cell = (int(round(candidate.x)), int(round(candidate.y)))
                if grid.is_free(*cell):
                    inflate_cell(grid, cell[0], cell[1],
                                 args.planner_robot_radius)
                    dynamic_obstacle = cell
                    simulator.add_static_obstacle(cell[0], cell[1])

            if dynamic_obstacle is not None:
                blocked = any(
                    int(round(point.x)) == dynamic_obstacle[0] and
                    int(round(point.y)) == dynamic_obstacle[1]
                    for point in current_path)
                if blocked:
                    observed = simulator.observe()
                    current_cell = autoplanner.Point2i(
                        int(round(observed["x"])),
                        int(round(observed["y"])))
                    start_time = time.perf_counter()
                    replanned_result = dstar.replan(grid, current_cell)
                    dstar_time_ms = (time.perf_counter() - start_time) * 1000.0
                    replanning_count += 1
                    replanning_time_ms += dstar_time_ms
                    if replanned_result.success:
                        current_path = list(replanned_result.path)
                        trajectory = make_trajectory(current_path, args)
                        replanned = True
                        if args.controller == "mpc":
                            controller.reset_reference_progress()
                    else:
                        safe_stop = True

            for step in range(args.steps_per_frame):
                observed = simulator.observe()
                trajectory_index, reference = closest_trajectory(
                    trajectory, observed["x"], observed["y"])
                state = autompc.State(
                    observed["x"], observed["y"],
                    observed["theta"], observed["v"])
                if safe_stop:
                    command = autompc.Control(0.0, 0.0)
                elif args.controller == "stanley":
                    command = controller.compute(state, reference, reference.v)
                else:
                    command = controller.compute(state, trajectory, reference.v)
                next_state = simulator.step(command.velocity, command.steering)
                cross_track = abs(
                    -math.sin(reference.theta) *
                    (next_state["x"] - reference.x)
                    + math.cos(reference.theta) *
                    (next_state["y"] - reference.y))
                goal_distance = math.hypot(
                    next_state["x"] - trajectory[-1].x,
                    next_state["y"] - trajectory[-1].y)
                obstacle_contacts = int(next_state["obstacle_contacts"])
                collision_steps += int(obstacle_contacts > 0)
                rows.append({
                    "frame": frame, "step": step,
                    "x": next_state["x"], "y": next_state["y"],
                    "theta": next_state["theta"], "v": next_state["v"],
                    "ref_x": reference.x, "ref_y": reference.y,
                    "ref_theta": reference.theta,
                    "command_velocity": command.velocity,
                    "command_steering": command.steering,
                    "replanned": int(replanned and step == 0),
                    "dynamic_obstacle_x": dynamic_obstacle[0]
                    if dynamic_obstacle else -1,
                    "dynamic_obstacle_y": dynamic_obstacle[1]
                    if dynamic_obstacle else -1,
                    "dstar_time_ms": dstar_time_ms if step == 0 else 0.0,
                    "cross_track": cross_track,
                    "goal_distance": goal_distance,
                    "obstacle_contacts": obstacle_contacts,
                    "safe_stop": int(safe_stop),
                })
                if (trajectory_index >= len(trajectory) - 5 and
                        goal_distance <= 0.75):
                    goal_reached = True
                    break
            if goal_reached or safe_stop:
                break
    finally:
        simulator.close()

    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    summary = {
        "map": str(map_path), "controller": args.controller,
        "physics_model": "racecar", "wheelbase": args.wheelbase,
        "frames_requested": args.frames,
        "frames_run": max((int(row["frame"]) for row in rows), default=-1) + 1,
        "replanning_count": replanning_count,
        "total_replanning_time_ms": replanning_time_ms,
        "dynamic_obstacle": dynamic_obstacle,
        "collision_steps": collision_steps,
        "safe_stop": safe_stop, "goal_reached": goal_reached,
        "final_goal_distance": rows[-1]["goal_distance"] if rows else None,
        "csv": str(csv_path),
    }
    summary_path.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
