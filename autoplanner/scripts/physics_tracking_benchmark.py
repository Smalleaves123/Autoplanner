#!/usr/bin/env python3
"""Closed-loop planner/controller/physics-engine benchmark.

The C++ planner produces a waypoint path, the C++ trajectory layer produces a
curvature-aware reference, and a C++ Stanley or MPC controller is evaluated
against a MuJoCo or PyBullet execution backend. Raw per-step CSV and summary
JSON files are written for every run.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

from physics_backend_smoke import (  # type: ignore
    MujocoBicycleSimulator,
    PhysicsOptions,
    PyBulletBicycleSimulator,
    PyBulletRacecarSimulator,
)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_obstacle_rectangles(path: Path) -> list[tuple[float, float, float, float]]:
    """Merge horizontal occupied-cell runs into static collision boxes."""
    rectangles = []
    for y, raw_line in enumerate(path.read_text().splitlines()):
        line = raw_line.strip()
        run_start = None
        for x in range(len(line) + 1):
            occupied = x < len(line) and line[x] in "1#@"
            if occupied and run_start is None:
                run_start = x
            if not occupied and run_start is not None:
                run_end = x - 1
                rectangles.append((
                    0.5 * (run_start + run_end), float(y),
                    0.5 * (run_end - run_start + 1), 0.5))
                run_start = None
    return rectangles


def planner_path(args: argparse.Namespace, root: Path) -> tuple[Path, dict[str, Any]]:
    if args.path:
        path = Path(args.path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(path)
        return path, {"source": "existing", "path": str(path)}

    planner_cli = (root / args.build_dir / "apps" / "autoplanner_cli").resolve()
    if not planner_cli.exists():
        raise FileNotFoundError(f"planner executable not found: {planner_cli}")
    map_path = (root / args.map).resolve()
    with tempfile.TemporaryDirectory(prefix="robotnav-physics-plan-") as temp:
        output_dir = Path(temp)
        command = [
            str(planner_cli), "--planner", args.planner,
            "--map", str(map_path),
            "--start", str(args.start[0]), str(args.start[1]),
            "--goal", str(args.goal[0]), str(args.goal[1]),
            "--smooth", "shortcut", "--output", str(output_dir),
        ]
        if args.planner == "improved_astar":
            command.extend((
                "--robot-radius", str(args.planner_robot_radius),
                "--footprint", "rectangle",
                "--robot-length", "0.65",
                "--robot-width", "0.20",
                "--inflate"))
        result = subprocess.run(command, text=True, capture_output=True)
        path = output_dir / "path.csv"
        metrics = output_dir / "metrics.json"
        if result.returncode != 0 or not path.exists():
            raise RuntimeError(
                "planning failed\n" + result.stdout + "\n" + result.stderr)
        # Copy the path to a stable output location before the temp directory
        # is removed; the trajectory reference is generated from this file.
        stable_path = (root / args.output_dir / "planned_path.csv").resolve()
        stable_path.parent.mkdir(parents=True, exist_ok=True)
        stable_path.write_bytes(path.read_bytes())
        planner_metrics = json.loads(metrics.read_text()) if metrics.exists() else {}
        return stable_path, {"source": "planner", **planner_metrics}


def nearest_reference(trajectory: list[Any], state: dict[str, float]) -> tuple[int, Any]:
    index = min(
        range(len(trajectory)),
        key=lambda i: (trajectory[i].x - state["x"]) ** 2
        + (trajectory[i].y - state["y"]) ** 2,
    )
    return index, trajectory[index]


def run(args: argparse.Namespace, backend_name: str, path: Path,
        planner_metrics: dict[str, Any], root: Path) -> dict[str, Any]:
    import autompc

    wheelbase = args.wheelbase
    if wheelbase is None:
        wheelbase = 0.325 if (
            backend_name == "pybullet" and args.pybullet_model == "racecar"
        ) else 1.0
    trajectory_options = autompc.TrajectoryOptions()
    trajectory_options.sample_spacing = args.sample_spacing
    trajectory_options.target_velocity = args.velocity
    trajectory_options.max_velocity = args.max_velocity
    trajectory_options.max_acceleration = args.max_acceleration
    trajectory_options.max_deceleration = args.max_deceleration
    trajectory_options.max_lateral_acceleration = args.max_lateral_acceleration
    trajectory = autompc.load_path_csv_with_options(
        str(path), args.velocity, trajectory_options)
    if not trajectory:
        raise RuntimeError("generated trajectory is empty")

    physics_options = PhysicsOptions(
        dt=args.dt,
        wheelbase=wheelbase,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        max_deceleration=args.max_deceleration,
        max_steering=args.max_steering,
        max_steering_rate=args.max_steering_rate,
    )
    if backend_name == "pybullet" and args.pybullet_model == "racecar":
        physics_options.obstacle_rectangles = load_obstacle_rectangles(
            (root / args.map).resolve())
    if backend_name == "mujoco":
        simulator_class = MujocoBicycleSimulator
    elif args.pybullet_model == "racecar":
        simulator_class = PyBulletRacecarSimulator
    else:
        simulator_class = PyBulletBicycleSimulator
    simulator = simulator_class(physics_options)
    initial = trajectory[0]
    simulator.reset(initial.x, initial.y + args.initial_offset,
                    initial.theta, 0.0)

    if args.controller == "stanley":
        controller = autompc.StanleyController(0.5, wheelbase)
    else:
        controller = autompc.MPCController(
            args.mpc_horizon, args.dt, wheelbase,
            args.max_velocity, args.max_steering,
            args.max_acceleration, args.max_deceleration,
            args.max_steering_rate)

    output_dir = (root / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{backend_name}_{args.controller}.csv"
    json_path = output_dir / f"{backend_name}_{args.controller}.json"
    fields = [
        "step", "x", "y", "theta", "v", "ref_x", "ref_y", "ref_theta",
        "ref_v", "command_velocity", "command_steering", "cross_track",
        "heading_error", "goal_distance", "reference_index",
        "obstacle_contacts",
    ]
    rows: list[dict[str, float | int]] = []
    actual_path_length = 0.0
    previous_state: dict[str, float] | None = None
    max_cross_track = 0.0
    max_heading_error = 0.0
    collision_steps = 0
    goal_reached = False
    try:
        for step in range(args.steps):
            current = simulator.observe()
            reference_index, reference = nearest_reference(trajectory, current)
            state_object = autompc.State(
                current["x"], current["y"], current["theta"], current["v"])
            if args.controller == "stanley":
                command = controller.compute(
                    state_object, reference, reference.v)
            else:
                command = controller.compute(
                    state_object, trajectory, reference.v)
            next_state = simulator.step(command.velocity, command.steering)
            if previous_state is not None:
                actual_path_length += math.hypot(
                    next_state["x"] - previous_state["x"],
                    next_state["y"] - previous_state["y"])
            previous_state = next_state
            cross_track = abs(
                -math.sin(reference.theta) * (next_state["x"] - reference.x)
                + math.cos(reference.theta) * (next_state["y"] - reference.y))
            heading_error = abs(wrap_angle(next_state["theta"] - reference.theta))
            goal_distance = math.hypot(
                next_state["x"] - trajectory[-1].x,
                next_state["y"] - trajectory[-1].y)
            max_cross_track = max(max_cross_track, cross_track)
            max_heading_error = max(max_heading_error, heading_error)
            obstacle_contacts = int(next_state.get("obstacle_contacts", 0.0))
            collision_steps += int(obstacle_contacts > 0)
            rows.append({
                "step": step, "x": next_state["x"], "y": next_state["y"],
                "theta": next_state["theta"], "v": next_state["v"],
                "ref_x": reference.x, "ref_y": reference.y,
                "ref_theta": reference.theta, "ref_v": reference.v,
                "command_velocity": command.velocity,
                "command_steering": command.steering,
                "cross_track": cross_track, "heading_error": heading_error,
                "goal_distance": goal_distance,
                "reference_index": reference_index,
                "obstacle_contacts": obstacle_contacts,
            })
            if (reference_index >= len(trajectory) - 5 and
                    goal_distance <= args.goal_tolerance):
                goal_reached = True
                break
    finally:
        close = getattr(simulator, "close", None)
        if close is not None:
            close()

    with csv_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    final = rows[-1] if rows else {}
    summary = {
        "backend": backend_name,
        "physics_model": (
            args.pybullet_model if backend_name == "pybullet"
            else "planar_mujoco"),
        "controller": args.controller,
        "wheelbase": wheelbase,
        "path": str(path),
        "planner": planner_metrics,
        "steps": len(rows),
        "goal_reached": goal_reached,
        "goal_distance": final.get("goal_distance", 0.0),
        "actual_path_length": actual_path_length,
        "max_cross_track": max_cross_track,
        "mean_cross_track": (
            sum(float(row["cross_track"]) for row in rows) / len(rows)
            if rows else 0.0),
        "max_heading_error": max_heading_error,
        "mean_heading_error": (
            sum(float(row["heading_error"]) for row in rows) / len(rows)
            if rows else 0.0),
        "collision_steps": collision_steps,
        "run_success": True,
        "csv": str(csv_path),
    }
    json_path.write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "pybullet", "both"),
                        default="both")
    parser.add_argument("--pybullet-model", choices=("racecar", "planar"),
                        default="racecar")
    parser.add_argument("--controller", choices=("stanley", "mpc"),
                        default="stanley")
    parser.add_argument("--path")
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--planner", default="improved_astar")
    parser.add_argument(
        "--planner-robot-radius", type=float, default=0.4,
        help="physical footprint radius passed to the planner")
    parser.add_argument("--start", nargs=2, type=int, default=(1, 1))
    parser.add_argument("--goal", nargs=2, type=int, default=(48, 48))
    parser.add_argument("--output-dir",
                        default="autoplanner/results/physics_tracking")
    parser.add_argument("--steps", type=int, default=2200)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument(
        "--wheelbase", type=float, default=None,
        help="vehicle wheelbase; defaults to 0.325 for racecar, 1.0 otherwise")
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-steering", type=float, default=0.7)
    parser.add_argument("--max-acceleration", type=float, default=1.5)
    parser.add_argument("--max-deceleration", type=float, default=2.0)
    parser.add_argument("--max-steering-rate", type=float, default=1.5)
    parser.add_argument("--mpc-horizon", type=int, default=15)
    parser.add_argument("--sample-spacing", type=float, default=0.5)
    parser.add_argument("--max-lateral-acceleration", type=float, default=1.5)
    parser.add_argument("--initial-offset", type=float, default=0.5)
    parser.add_argument("--goal-tolerance", type=float, default=0.75)
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")

    root = Path(__file__).resolve().parents[2]
    path, planner_metrics = planner_path(args, root)
    backends = ("mujoco", "pybullet") if args.backend == "both" else (args.backend,)
    summaries = {}
    for backend in backends:
        summaries[backend] = run(args, backend, path, planner_metrics, root)
        print(json.dumps(summaries[backend], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
