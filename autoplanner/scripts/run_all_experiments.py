#!/usr/bin/env python3
"""Run reproducible planning, tracking, and dynamic-replanning benchmarks.

Every run keeps its raw CLI artifacts (path, trajectory, tracking CSV, and
JSON metrics). The combined CSV files are summaries of those real artifacts,
not estimates derived from documented algorithm properties.
"""
import argparse
import csv
import json
import math
import os
import subprocess
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
AUTOPLANNER_DIR = SCRIPT_DIR.parent
REPO_ROOT = AUTOPLANNER_DIR.parent

PLANNERS = {
    "dijkstra": [],
    "astar": [],
    "weighted_astar": ["--weight", "1.5"],
    "improved_astar": ["--robot-radius", "1.0"],
    "jps": [],
    "dstar_lite": [],
    "rrt": [],
    "rrt_star": [],
    "informed_rrt_star": [],
    "bi_rrt": [],
    "hybrid_astar": [],
}

MAPS = [
    ("simple_50x50", "maps/simple_50x50.txt", (1, 1), (48, 48)),
    ("maze_100x100", "maps/maze_100x100.txt", (1, 1), (98, 98)),
    ("warehouse_100x100", "maps/warehouse_100x100.txt", (3, 5), (90, 80)),
    ("random_100x100_density_20", "maps/random_100x100_density_20.txt",
     (1, 1), (98, 98)),
]


def read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def bool_value(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes")


def number(metrics: dict, key: str, default=0.0):
    try:
        return float(metrics.get(key, default))
    except (TypeError, ValueError):
        return default


def integer(metrics: dict, key: str, default=0):
    try:
        return int(metrics.get(key, default))
    except (TypeError, ValueError):
        return default


def path_length(path_file: Path) -> float:
    points = []
    with path_file.open() as stream:
        next(stream, None)
        for line in stream:
            x_text, y_text = line.strip().split(",")[:2]
            points.append((float(x_text), float(y_text)))
    return sum(math.hypot(x1 - x0, y1 - y0)
               for (x0, y0), (x1, y1) in zip(points, points[1:]))


def run_planner(cli: Path, planner: str, extra_args: list[str],
                map_path: Path, map_name: str, start: tuple[int, int],
                goal: tuple[int, int], output_dir: Path,
                smoothing: str) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(cli), "--planner", planner, "--map", str(map_path),
        "--start", str(start[0]), str(start[1]),
        "--goal", str(goal[0]), str(goal[1]),
        "--smooth", smoothing, "--output", str(output_dir),
        *extra_args,
    ]
    try:
        process = subprocess.run(command, capture_output=True, text=True,
                                 timeout=120)
    except (OSError, subprocess.TimeoutExpired) as error:
        process = None
        error_text = str(error)
    else:
        error_text = process.stderr.strip()

    metrics = read_json(output_dir / "metrics.json")
    row = {
        "planner": planner,
        "map": map_name,
        "success": bool_value(metrics.get("success", False)),
        "collision_free": bool_value(metrics.get("collision_free", False)),
        "planning_time_ms": number(metrics, "planning_time_ms"),
        "path_length": number(metrics, "path_length"),
        "expanded_nodes": integer(metrics, "expanded_nodes"),
        "path_points": integer(metrics, "path_points"),
        "turning_count": integer(metrics, "turning_count"),
        "total_turning": number(metrics, "total_turning"),
        "average_curvature": number(metrics, "average_curvature"),
        "smoothness_score": number(metrics, "smoothness_score"),
        "minimum_obstacle_distance": number(
            metrics, "minimum_obstacle_distance"),
        "return_code": process.returncode if process is not None else -1,
        "output_dir": str(output_dir),
        "error": error_text,
    }
    if process is None or process.returncode != 0:
        row["success"] = False
    return row


def run_tracking(cli: Path, planner: str, map_name: str, repeat: int,
                 path_file: Path, controller: str, output_dir: Path,
                 velocity: float, dt: float, sample_spacing: float,
                 max_lateral_acceleration: float) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    length = path_length(path_file)
    steps = max(100, math.ceil(1.5 * length / max(velocity * dt, 1e-6)) + 100)
    command = [
        str(cli), "--controller", controller, "--trajectory", "path",
        "--path", str(path_file), "--velocity", str(velocity),
        "--steps", str(steps), "--dt", str(dt),
        "--sample-spacing", str(sample_spacing),
        "--max-lateral-acceleration", str(max_lateral_acceleration),
        "--output", str(output_dir / "tracking.csv"),
        "--metrics", str(output_dir / "tracking_metrics.json"),
        "--trajectory-output", str(output_dir / "trajectory.csv"),
    ]
    if controller == "mpc":
        command += ["--max-velocity", "2.0", "--max-acceleration", "1.5",
                    "--max-deceleration", "2.0", "--mpc-horizon", "15"]

    try:
        process = subprocess.run(command, capture_output=True, text=True,
                                 timeout=120)
    except (OSError, subprocess.TimeoutExpired) as error:
        process = None
        error_text = str(error)
    else:
        error_text = process.stderr.strip()

    metrics = read_json(output_dir / "tracking_metrics.json")
    return {
        "planner": planner,
        "map": map_name,
        "repeat": repeat,
        "controller": controller,
        "run_success": process is not None and process.returncode == 0,
        "goal_reached": bool_value(metrics.get("goal_reached", False)),
        "steps": integer(metrics, "steps"),
        "max_cross_track": number(metrics, "max_cross_track"),
        "mean_cross_track": number(metrics, "mean_cross_track"),
        "max_heading_error": number(metrics, "max_heading_error"),
        "mean_heading_error": number(metrics, "mean_heading_error"),
        "goal_distance": number(metrics, "goal_distance"),
        "output_dir": str(output_dir),
        "error": error_text,
    }


def run_dynamic_replanning(cli: Path, map_name: str, map_path: Path,
                           repeat: int, output_dir: Path, frames: int,
                           controller: str) -> dict:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "dynamic_replanning.csv"
    summary_path = output_dir / "summary.json"
    command = [str(cli), "--map", str(map_path), "--frames", str(frames),
               "--steps-per-frame", "40", "--controller", controller,
               "--output", str(csv_path), "--summary", str(summary_path)]
    try:
        process = subprocess.run(command, capture_output=True, text=True,
                                 timeout=120)
    except (OSError, subprocess.TimeoutExpired) as error:
        process = None
        error_text = str(error)
    else:
        error_text = process.stderr.strip()

    summary = read_json(summary_path)
    return {
        "map": map_name,
        "repeat": repeat,
        "run_success": process is not None and bool(summary),
        "controller": controller,
        "frames_requested": integer(summary, "frames_requested", frames),
        "frames_run": integer(summary, "frames_run"),
        "final_success": bool_value(summary.get("final_success", False)),
        "safe_stop": bool_value(summary.get("safe_stop", False)),
        "safe_stop_steps": integer(summary, "safe_stop_steps"),
        "safe_stop_distance": number(summary, "safe_stop_distance"),
        "replanning_count": integer(
            summary, "replanning_count",
            integer(summary, "replanning_attempts")),
        "dstar_total_time_ms": number(summary, "dstar_total_time_ms"),
        "astar_total_time_ms": number(summary, "astar_total_time_ms"),
        "dstar_over_astar_speedup": number(
            summary, "dstar_over_astar_speedup"),
        "mean_steering_delta": number(summary, "mean_steering_delta"),
        "max_steering_delta": number(summary, "max_steering_delta"),
        "mean_velocity_delta": number(summary, "mean_velocity_delta"),
        "max_velocity_delta": number(summary, "max_velocity_delta"),
        "mean_replan_steering_delta": number(
            summary, "mean_replan_steering_delta"),
        "max_replan_steering_delta": number(
            summary, "max_replan_steering_delta"),
        "mean_replan_velocity_delta": number(
            summary, "mean_replan_velocity_delta"),
        "max_replan_velocity_delta": number(
            summary, "max_replan_velocity_delta"),
        "output_dir": str(output_dir),
        "error": error_text,
    }


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run RobotNav benchmarks")
    parser.add_argument("--build_dir", default="build")
    parser.add_argument("--data_dir", default=str(AUTOPLANNER_DIR / "data"))
    parser.add_argument("--output_dir", default="results/benchmark")
    parser.add_argument("--planners", nargs="*", default=list(PLANNERS))
    parser.add_argument("--controllers", nargs="*",
                        default=["stanley", "pure_pursuit", "mpc"])
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--smooth", choices=("none", "shortcut"), default="shortcut")
    parser.add_argument("--dynamic-frames", type=int, default=5)
    parser.add_argument("--dynamic-controller", choices=("stanley", "mpc"),
                        default="mpc")
    parser.add_argument("--skip-dynamic", action="store_true")
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--sample-spacing", type=float, default=0.5)
    parser.add_argument("--max-lateral-acceleration", type=float, default=1.5)
    args = parser.parse_args()

    build_dir = Path(args.build_dir).expanduser().resolve()
    data_root = Path(args.data_dir).expanduser().resolve()
    output_root = Path(args.output_dir).expanduser().resolve()
    planner_cli = build_dir / "apps" / "autoplanner_cli"
    tracker_cli = build_dir / "apps" / "autompc_cli"
    dynamic_cli = build_dir / "apps" / "dynamic_replanning"
    dynamic_navigation_cli = build_dir / "apps" / "dynamic_navigation"
    if not planner_cli.exists() or not tracker_cli.exists():
        print("Build executables not found; build the project first.", file=sys.stderr)
        return 1

    output_root.mkdir(parents=True, exist_ok=True)
    planning_rows = []
    tracking_rows = []
    dynamic_rows = []

    for repeat in range(args.repeat):
        for planner in args.planners:
            for map_name, relative_map, start, goal in MAPS:
                map_path = data_root / relative_map
                if not map_path.exists():
                    print(f"[SKIP] map not found: {map_path}")
                    continue
                run_dir = output_root / f"{planner}_{map_name}_{repeat}"
                print(f"[planning] {planner} on {map_name} repeat={repeat}")
                row = run_planner(
                    planner_cli, planner, PLANNERS.get(planner, []), map_path,
                    map_name, start, goal, run_dir / "planning", args.smooth)
                row["repeat"] = repeat
                planning_rows.append(row)

                if not row["success"]:
                    continue
                path_file = run_dir / "planning" / "path.csv"
                for controller in args.controllers:
                    print(f"[tracking] {controller} on {planner}/{map_name}")
                    tracking_rows.append(run_tracking(
                        tracker_cli, planner, map_name, repeat, path_file,
                        controller, run_dir / "tracking" / controller,
                        args.velocity, args.dt, args.sample_spacing,
                        args.max_lateral_acceleration))

        dynamic_runner = (dynamic_navigation_cli
                          if dynamic_navigation_cli.exists() else dynamic_cli)
        if not args.skip_dynamic and dynamic_runner.exists():
            for map_name, relative_map, _, _ in MAPS:
                map_path = data_root / relative_map
                if map_path.exists():
                    print(f"[dynamic] {map_name} repeat={repeat}")
                    dynamic_rows.append(run_dynamic_replanning(
                        dynamic_runner, map_name, map_path, repeat,
                        output_root / f"dynamic_{map_name}_{repeat}",
                        args.dynamic_frames, args.dynamic_controller))

    planning_fields = [
        "repeat", "planner", "map", "success", "collision_free",
        "planning_time_ms", "path_length", "expanded_nodes", "path_points",
        "turning_count", "total_turning", "average_curvature",
        "smoothness_score", "minimum_obstacle_distance", "return_code",
        "output_dir", "error",
    ]
    tracking_fields = [
        "repeat", "planner", "map", "controller", "run_success",
        "goal_reached", "steps", "max_cross_track", "mean_cross_track",
        "max_heading_error", "mean_heading_error", "goal_distance",
        "output_dir", "error",
    ]
    dynamic_fields = [
        "repeat", "map", "controller", "run_success", "frames_requested",
        "frames_run", "final_success", "safe_stop", "safe_stop_steps",
        "safe_stop_distance", "replanning_count",
        "dstar_total_time_ms", "astar_total_time_ms",
        "dstar_over_astar_speedup", "mean_steering_delta",
        "max_steering_delta", "mean_velocity_delta", "max_velocity_delta",
        "mean_replan_steering_delta", "max_replan_steering_delta",
        "mean_replan_velocity_delta", "max_replan_velocity_delta",
        "output_dir", "error",
    ]
    write_csv(output_root / "planning_results.csv", planning_rows, planning_fields)
    write_csv(output_root / "all_results.csv", planning_rows, planning_fields)
    write_csv(output_root / "tracking_results.csv", tracking_rows, tracking_fields)
    write_csv(output_root / "dynamic_replanning_results.csv", dynamic_rows,
              dynamic_fields)

    manifest = {
        "smoothing": args.smooth,
        "planners": args.planners,
        "controllers": args.controllers,
        "dynamic_controller": args.dynamic_controller,
        "repeat": args.repeat,
        "dynamic_frames": args.dynamic_frames,
        "planning_rows": len(planning_rows),
        "tracking_rows": len(tracking_rows),
        "dynamic_rows": len(dynamic_rows),
        "files": ["planning_results.csv", "tracking_results.csv",
                  "dynamic_replanning_results.csv"],
    }
    (output_root / "benchmark_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n")
    print(f"\nBenchmark complete: {output_root}")
    print(f"Planning rows: {len(planning_rows)}")
    print(f"Tracking rows: {len(tracking_rows)}")
    print(f"Dynamic rows: {len(dynamic_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
