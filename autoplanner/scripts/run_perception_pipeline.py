#!/usr/bin/env python3
"""Run a ROS-free sensor -> map -> tracking -> planning experiment.

With no sensor file, a deterministic lidar simulator observes the selected
static map and optional constant-velocity circular obstacles.  Existing sensor
data can be replayed with ``--sensor-csv`` or ``--sensor-json`` using the
interchange format documented in the repository README.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

from perception_mapping import (
    LidarConfig,
    LidarSimulator,
    PerceptionMappingPipeline,
    SensorFrame,
    SimulatedObstacle,
    load_grid,
    load_sensor_frames,
    save_dynamic_updates,
    save_grid,
    save_sensor_csv,
    tracked_obstacle_updates,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_MAP = REPO_ROOT / "autoplanner" / "data" / "maps" / "simple_50x50.txt"


def _finite_or_blank(value: float) -> float | str:
    return value if math.isfinite(value) else ""


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _replay_fingerprint(output_dir: Path) -> str:
    """Hash replay artifacts whose contents do not contain wall-clock timing."""
    paths = [
        output_dir / "sensor_points.csv",
        output_dir / "cpp_dynamic_updates.csv",
        output_dir / "tracks.json",
        *sorted(output_dir.glob("occupancy_frame_*.txt")),
    ]
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _parse_obstacle(values: list[str]) -> SimulatedObstacle:
    obstacle_id, x, y, vx, vy, radius = values
    return SimulatedObstacle(obstacle_id, float(x), float(y),
                             float(vx), float(vy), float(radius))


def _resolve_path(value: str | Path) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    if path.exists():
        return path.resolve()
    return (REPO_ROOT / path).resolve()


def _run_cpp_dynamic_pipeline(args: argparse.Namespace,
                              map_path: Path,
                              output_dir: Path,
                              updates_path: Path,
                              frame_count: int) -> dict[str, object]:
    build_dir = _resolve_path(args.cpp_build_dir)
    pipeline_cli = build_dir / "apps" / "dynamic_navigation_pipeline_cli"
    if not pipeline_cli.exists():
        raise RuntimeError(
            f"C++ dynamic pipeline executable not found: {pipeline_cli}")
    cpp_output_dir = (_resolve_path(args.cpp_output_dir)
                      if args.cpp_output_dir
                      else output_dir / "cpp_dynamic")
    cpp_output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(pipeline_cli),
        "--map", str(map_path),
        "--planner", args.cpp_planner,
        "--controller", args.cpp_controller,
        "--local-planner", args.cpp_local_planner,
        "--start", str(args.start[0]), str(args.start[1]),
        "--goal", str(args.goal[0]), str(args.goal[1]),
        "--footprint", "point",
        "--smooth", "none",
        "--frames", str(max(1, frame_count)),
        "--steps-per-frame", str(args.cpp_steps_per_frame),
        "--no-auto-obstacles",
        "--perception-updates", str(updates_path),
        "--output-dir", str(cpp_output_dir),
    ]
    if args.cpp_planner == "space_time_astar":
        command += ["--prediction-horizon", str(args.cpp_prediction_horizon)]
    if args.cpp_local_planner == "dwa":
        command += [
            "--dwa-prediction-time", str(args.cpp_prediction_time),
            "--dwa-dynamic-collision-samples",
            str(args.cpp_dynamic_collision_samples),
        ]
    elif args.cpp_local_planner == "mppi":
        command += [
            "--mppi-prediction-time", str(args.cpp_prediction_time),
            "--mppi-horizon", str(args.cpp_mppi_horizon),
            "--mppi-rollouts", str(args.cpp_mppi_rollouts),
        ]
    print("Running C++ dynamic pipeline with perception updates...")
    completed = subprocess.run(command, text=True)
    metrics_path = cpp_output_dir / "metrics.json"
    trace_path = cpp_output_dir / "trace.csv"
    if not metrics_path.exists() or not trace_path.exists():
        raise RuntimeError(
            "C++ dynamic pipeline failed; see the C++ output above")
    if completed.returncode != 0:
        print(
            "C++ dynamic pipeline completed with a non-success navigation "
            f"status (return code {completed.returncode}); preserving artifacts."
        )
    return {
        "output_dir": str(cpp_output_dir),
        "trace": str(trace_path),
        "metrics": str(metrics_path),
        "command": command,
        "return_code": completed.returncode,
        "run_success": completed.returncode == 0,
        "result": json.loads(metrics_path.read_text()),
    }


def run(args: argparse.Namespace) -> dict[str, object]:
    map_path = _resolve_path(args.map)
    static_grid = load_grid(map_path)
    output_dir = _resolve_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    input_path = args.sensor_csv or args.sensor_json
    if input_path:
        sensor_input_path = _resolve_path(input_path)
        sensor_frames = load_sensor_frames(sensor_input_path)
        simulator = None
        source = str(sensor_input_path)
    else:
        obstacles = tuple(_parse_obstacle(values)
                          for values in (args.dynamic_obstacle or []))
        simulator = LidarSimulator(
            static_grid, obstacles,
            LidarConfig(args.beams, args.sensor_range, args.ray_step,
                        args.sensor_noise, args.seed),
        )
        sensor_frames = {}
        source = "simulator"

    pipeline = PerceptionMappingPipeline(
        static_grid, tuple(args.start), tuple(args.goal),
        unknown_policy=args.unknown_policy,
        initial_map_known=not args.initial_unknown,
        robot_radius=args.robot_radius,
    )
    robot = tuple(args.start)
    frames = sorted(sensor_frames) if input_path else list(range(args.frames))
    if args.frames > 0 and input_path:
        frames = frames[:args.frames]
    recorded_frames: list[SensorFrame] = []
    trace_rows: list[dict[str, object]] = []
    update_rows: list[dict[str, object]] = []
    track_rows: list[dict[str, object]] = []
    total_replans = 0
    total_detections = 0
    min_clearance = float("inf")
    perception_steps = []

    for frame in frames:
        if simulator is not None:
            sensor = simulator.scan(
                frame, robot[0] + 0.5, robot[1] + 0.5)
        else:
            sensor = sensor_frames[frame]
        recorded_frames.append(sensor)
        step = pipeline.step(sensor, robot)
        perception_steps.append(step)
        total_replans += int(step.replanned)
        total_detections += len(step.detections)
        if math.isfinite(step.minimum_path_clearance):
            min_clearance = min(min_clearance, step.minimum_path_clearance)
        for x, y in step.changed_cells:
            update_rows.append({"frame": frame, "cell_x": x, "cell_y": y})
        track_rows.append({
            "frame": frame,
            "detections": [
                {"frame": item.frame, "x": item.x, "y": item.y,
                 "radius": item.radius, "point_count": item.point_count}
                for item in step.detections
            ],
            "tracks": [track.as_dict() for track in step.tracks],
            "predictions": pipeline.tracker.predictions(args.prediction_frames),
        })
        save_grid(output_dir / f"occupancy_frame_{frame:03d}.txt",
                  pipeline.occupancy.to_grid(args.unknown_policy))
        trace_rows.append({
            "frame": frame,
            "robot_x": step.robot[0],
            "robot_y": step.robot[1],
            "goal_reached": int(step.robot == tuple(args.goal)),
            "changed_cells": len(step.changed_cells),
            "detections": len(step.detections),
            "tracks": len(step.tracks),
            "occupied_cells": step.occupied_cells,
            "unknown_cells": step.unknown_cells,
            "path_length": len(step.path),
            "replanned": int(step.replanned),
            "expanded_nodes": step.expanded_nodes,
            "planning_time_ms": step.planning_time_ms,
            "minimum_path_clearance": _finite_or_blank(
                step.minimum_path_clearance),
        })
        if step.robot == tuple(args.goal):
            break
        if len(step.path) > 1:
            candidate = step.path[1]
            planning_grid = pipeline.occupancy.to_grid(args.unknown_policy)
            if planning_grid[candidate[1]][candidate[0]] == 0:
                robot = candidate

    save_sensor_csv(output_dir / "sensor_points.csv", recorded_frames)
    with (output_dir / "trace.csv").open("w", newline="") as stream:
        fields = tuple(trace_rows[0]) if trace_rows else (
            "frame", "robot_x", "robot_y", "goal_reached", "changed_cells",
            "detections", "tracks", "occupied_cells", "unknown_cells",
            "path_length", "replanned", "expanded_nodes", "planning_time_ms",
            "minimum_path_clearance")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(trace_rows)
    with (output_dir / "map_updates.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=("frame", "cell_x", "cell_y"))
        writer.writeheader()
        writer.writerows(update_rows)
    (output_dir / "tracks.json").write_text(
        json.dumps(track_rows, indent=2, allow_nan=False) + "\n")
    cpp_updates = tracked_obstacle_updates(perception_steps)
    cpp_updates_path = output_dir / "cpp_dynamic_updates.csv"
    save_dynamic_updates(cpp_updates_path, cpp_updates)

    cpp_result = None
    if args.cpp_build_dir:
        cpp_result = _run_cpp_dynamic_pipeline(
            args, map_path, output_dir, cpp_updates_path, len(trace_rows))

    replay_fingerprint = _replay_fingerprint(output_dir)
    manifest = {
        "schema_version": 1,
        "map": str(map_path),
        "map_sha256": _sha256(map_path),
        "sensor_source": source,
        "sensor_input_sha256": (
            _sha256(_resolve_path(input_path)) if input_path else None
        ),
        "deterministic_replay_fingerprint": replay_fingerprint,
        "parameters": {
            "start": list(args.start),
            "goal": list(args.goal),
            "frames": args.frames,
            "beams": args.beams,
            "sensor_range": args.sensor_range,
            "ray_step": args.ray_step,
            "sensor_noise": args.sensor_noise,
            "seed": args.seed,
            "unknown_policy": args.unknown_policy,
            "initial_unknown": args.initial_unknown,
            "robot_radius": args.robot_radius,
            "prediction_frames": args.prediction_frames,
            "dynamic_obstacles": args.dynamic_obstacle or [],
        },
    }
    summary = {
        "sensor_source": source,
        "map": str(map_path),
        "frames_requested": args.frames,
        "frames_processed": len(trace_rows),
        "unknown_policy": args.unknown_policy,
        "initial_map_known": not args.initial_unknown,
        "incremental_replanning": True,
        "replanning_count": total_replans,
        "dynamic_detection_count": total_detections,
        "track_count": len(pipeline.tracker.tracks),
        "unknown_cells_final": len(pipeline.occupancy.unknown_cells()),
        "minimum_path_clearance": _finite_or_blank(min_clearance),
        "start": list(args.start),
        "goal": list(args.goal),
        "final_robot": list(robot),
        "goal_reached": robot == tuple(args.goal),
        "final_path": [list(cell) for cell in pipeline.path],
        "deterministic_replay_fingerprint": replay_fingerprint,
        "artifacts": {
            "sensor_points": str(output_dir / "sensor_points.csv"),
            "map_updates": str(output_dir / "map_updates.csv"),
            "cpp_dynamic_updates": str(cpp_updates_path),
            "trace": str(output_dir / "trace.csv"),
            "tracks": str(output_dir / "tracks.json"),
            "manifest": str(output_dir / "replay_manifest.json"),
        },
    }
    if cpp_result is not None:
        summary["cpp_dynamic_pipeline"] = cpp_result
        manifest["cpp_dynamic_pipeline"] = {
            "build_dir": str(_resolve_path(args.cpp_build_dir)),
            "planner": args.cpp_planner,
            "controller": args.cpp_controller,
            "local_planner": args.cpp_local_planner,
            "steps_per_frame": args.cpp_steps_per_frame,
            "prediction_time": args.cpp_prediction_time,
            "dynamic_collision_samples": args.cpp_dynamic_collision_samples,
            "prediction_horizon": args.cpp_prediction_horizon,
            "mppi_horizon": args.cpp_mppi_horizon,
            "mppi_rollouts": args.cpp_mppi_rollouts,
            "command": cpp_result["command"],
        }
    (output_dir / "replay_manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n")
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default=str(DEFAULT_MAP))
    parser.add_argument("--start", nargs=2, type=int, default=(1, 1))
    parser.add_argument("--goal", nargs=2, type=int, default=(20, 20))
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--beams", type=int, default=72)
    parser.add_argument("--sensor-range", type=float, default=15.0)
    parser.add_argument("--ray-step", type=float, default=0.25)
    parser.add_argument("--sensor-noise", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--dynamic-obstacle", nargs=6, action="append", metavar=(
            "ID", "X", "Y", "VX", "VY", "RADIUS"),
        help="simulated obstacle; repeat for multiple obstacles")
    sensor_group = parser.add_mutually_exclusive_group()
    sensor_group.add_argument("--sensor-csv")
    sensor_group.add_argument("--sensor-json")
    parser.add_argument("--unknown-policy", choices=("occupied", "free"),
                        default="occupied")
    parser.add_argument("--initial-unknown", action="store_true",
                        help="start with an entirely unknown map")
    parser.add_argument("--robot-radius", type=float, default=0.0)
    parser.add_argument("--prediction-frames", type=int, default=5)
    parser.add_argument("--output-dir", default="autoplanner/results/perception_pipeline")
    parser.add_argument(
        "--cpp-build-dir",
        help="also replay tracked updates through the C++ dynamic pipeline")
    parser.add_argument("--cpp-output-dir")
    parser.add_argument("--cpp-planner", default="astar",
                        choices=("astar", "dstar_lite", "space_time_astar"))
    parser.add_argument("--cpp-controller", default="stanley",
                        choices=("pid", "pure_pursuit", "stanley", "mpc"))
    parser.add_argument("--cpp-local-planner", default="dwa",
                        choices=("none", "dwa", "mppi"))
    parser.add_argument("--cpp-steps-per-frame", type=int, default=20)
    parser.add_argument("--cpp-prediction-time", type=float, default=0.8)
    parser.add_argument("--cpp-dynamic-collision-samples", type=int,
                        default=3)
    parser.add_argument("--cpp-prediction-horizon", type=int, default=120)
    parser.add_argument("--cpp-mppi-horizon", type=int, default=12)
    parser.add_argument("--cpp-mppi-rollouts", type=int, default=32)
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")
    if args.prediction_frames < 0:
        parser.error("--prediction-frames must be non-negative")
    if args.cpp_steps_per_frame <= 0:
        parser.error("--cpp-steps-per-frame must be positive")
    if args.cpp_prediction_time <= 0.0:
        parser.error("--cpp-prediction-time must be positive")
    if args.cpp_dynamic_collision_samples <= 0:
        parser.error("--cpp-dynamic-collision-samples must be positive")
    if args.cpp_prediction_horizon <= 0:
        parser.error("--cpp-prediction-horizon must be positive")
    if args.cpp_mppi_horizon <= 0 or args.cpp_mppi_rollouts <= 0:
        parser.error("MPPI horizon and rollouts must be positive")
    try:
        summary = run(args)
    except (OSError, RuntimeError, ValueError, KeyError,
            json.JSONDecodeError) as error:
        print(f"Perception pipeline failed: {error}", file=sys.stderr)
        return 1
    print(json.dumps(summary, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
