#!/usr/bin/env python3
"""Benchmark RobotNav local planners in the reusable C++ pipelines.

The benchmark compares the nominal controller, DWA, and MPPI around the same
global planner, map, start/goal pair, and simulation settings.  It also runs a
small moving-obstacle scenario through the dynamic pipeline.  Every row is
backed by the pipeline's raw ``metrics.json`` file; no algorithm properties
are inferred by the Python runner.

Example from the repository root::

    python autoplanner/scripts/benchmark_local_planners.py \
        --build_dir build --repeat 2 \
        --output_dir autoplanner/results/local_planner_benchmark

The default benchmark is intentionally small enough for local development.
Use ``--scenarios`` and ``--skip-dynamic`` to narrow it further, or increase
``--mppi-rollouts`` when measuring controller quality instead of latency.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
import sys
from collections import defaultdict
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
AUTOPLANNER_DIR = SCRIPT_DIR.parent
REPO_ROOT = AUTOPLANNER_DIR.parent

LOCAL_PLANNERS = ("none", "dwa", "mppi")
CONTROLLERS = ("stanley", "mpc")

# Keep the default cases representative but quick.  The moving-obstacle case
# mirrors the dynamic pipeline's existing regression scenario.
SCENARIOS = {
    "simple": {
        "map": "maps/simple_50x50.txt",
        "start": (1, 1),
        "goal": (20, 20),
    },
    "warehouse": {
        "map": "maps/warehouse_100x100.txt",
        "start": (3, 5),
        "goal": (40, 35),
    },
}


def resolve_path(value: str | Path, base: Path = REPO_ROOT) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path.resolve()
    if path.exists():
        return path.resolve()
    return (base / path).resolve()


def read_json(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return {}


def bool_value(value: object) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes")


def number(metrics: dict, key: str, default: float = 0.0) -> float:
    try:
        return float(metrics.get(key, default))
    except (TypeError, ValueError):
        return default


def integer(metrics: dict, key: str, default: int = 0) -> int:
    try:
        return int(metrics.get(key, default))
    except (TypeError, ValueError):
        return default


def mppi_args(args: argparse.Namespace) -> list[str]:
    return [
        "--mppi-prediction-time", str(args.mppi_prediction_time),
        "--mppi-horizon", str(args.mppi_horizon),
        "--mppi-rollouts", str(args.mppi_rollouts),
        "--mppi-temperature", str(args.mppi_temperature),
        "--mppi-velocity-noise", str(args.mppi_velocity_noise),
        "--mppi-steering-noise", str(args.mppi_steering_noise),
    ]


def planner_args(local_planner: str, args: argparse.Namespace) -> list[str]:
    if local_planner == "mppi":
        return mppi_args(args)
    if local_planner == "dwa":
        return [
            "--dwa-prediction-time", str(args.dwa_prediction_time),
            "--dwa-velocity-samples", str(args.dwa_velocity_samples),
            "--dwa-steering-samples", str(args.dwa_steering_samples),
            "--dwa-dynamic-collision-samples",
            str(args.dwa_dynamic_collision_samples),
        ]
    return []


def run_command(command: list[str], timeout: int) -> tuple[int, str]:
    try:
        process = subprocess.run(
            command,
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        return -1, str(error)

    if process.returncode != 0:
        detail = process.stderr.strip() or process.stdout.strip()
        return process.returncode, detail[-2000:]
    return 0, ""


def static_command(
    cli: Path,
    scenario_file: Path,
    map_path: Path,
    start: tuple[int, int],
    goal: tuple[int, int],
    controller: str,
    local_planner: str,
    output_dir: Path,
    args: argparse.Namespace,
) -> list[str]:
    command = [
        str(cli),
        "--scenario", str(scenario_file),
        "--map", str(map_path),
        "--planner", args.planner,
        "--controller", controller,
        "--start", str(start[0]), str(start[1]),
        "--goal", str(goal[0]), str(goal[1]),
        "--local-planner", local_planner,
        "--smooth", args.smooth,
        "--max-steps", str(args.max_steps),
        "--velocity", str(args.velocity),
        "--dt", str(args.dt),
        "--output-dir", str(output_dir),
        *planner_args(local_planner, args),
    ]
    return command


def dynamic_command(
    cli: Path,
    scenario_file: Path,
    map_path: Path,
    start: tuple[int, int],
    goal: tuple[int, int],
    controller: str,
    local_planner: str,
    output_dir: Path,
    args: argparse.Namespace,
) -> list[str]:
    end_frame = max(3, args.frames // 3)
    command = [
        str(cli),
        "--scenario", str(scenario_file),
        "--map", str(map_path),
        "--planner", args.dynamic_planner,
        "--controller", controller,
        "--start", str(start[0]), str(start[1]),
        "--goal", str(goal[0]), str(goal[1]),
        "--local-planner", local_planner,
        "--smooth", args.smooth,
        "--velocity", str(args.velocity),
        "--frames", str(args.frames),
        "--steps-per-frame", str(args.steps_per_frame),
        "--no-auto-obstacles",
        "--moving-obstacle", "1", str(end_frame), "3", "10", "1", "0",
        "--moving-obstacle-radius", str(args.moving_obstacle_radius),
        "--moving-obstacle-uncertainty-growth",
        str(args.moving_obstacle_uncertainty_growth),
        "--prediction-risk-weight", str(args.prediction_risk_weight),
        "--prediction-risk-clearance", str(args.prediction_risk_clearance),
        "--output-dir", str(output_dir),
        *planner_args(local_planner, args),
    ]
    if args.moving_obstacle_acceleration != (0.0, 0.0):
        command += [
            "--moving-obstacle-acceleration",
            *(str(value) for value in args.moving_obstacle_acceleration),
        ]
    if args.moving_obstacle_covariance != (0.0, 0.0, 0.0):
        command += [
            "--moving-obstacle-covariance",
            *(str(value) for value in args.moving_obstacle_covariance),
        ]
    if args.moving_obstacle_covariance_growth != (0.0, 0.0, 0.0):
        command += [
            "--moving-obstacle-covariance-growth",
            *(str(value) for value in args.moving_obstacle_covariance_growth),
        ]
    if args.moving_obstacle_confidence_scale != 2.0:
        command += [
            "--moving-obstacle-confidence-scale",
            str(args.moving_obstacle_confidence_scale),
        ]
    return command


def static_row(
    metrics: dict,
    return_code: int,
    scenario_name: str,
    map_path: Path,
    controller: str,
    local_planner: str,
    output_dir: Path,
    error: str,
) -> dict:
    return {
        "scenario": scenario_name,
        "map": str(map_path),
        "controller": controller,
        "local_planner": local_planner,
        "success": bool_value(metrics.get("success", False)),
        "goal_reached": bool_value(metrics.get("goal_reached", False)),
        "collision_free": bool_value(metrics.get("collision_free", False)),
        "safe_stop": bool_value(metrics.get("safe_stop", False)),
        "planning_time_ms": number(metrics, "planning_time_ms"),
        "path_length": number(metrics, "path_length"),
        "controller_trace_steps": integer(metrics, "controller_trace_steps"),
        "local_planner_adjustments": integer(
            metrics, "local_planner_adjustments"),
        "local_planner_rollouts": integer(metrics, "local_planner_rollouts"),
        "local_planner_collision_rejections": integer(
            metrics, "local_planner_collision_rejections"),
        "local_planner_time_ms": number(metrics, "local_planner_time_ms"),
        "minimum_dynamic_obstacle_clearance": number(
            metrics, "minimum_dynamic_obstacle_clearance"),
        "mean_cross_track_error": number(metrics, "mean_cross_track_error"),
        "max_cross_track_error": number(metrics, "max_cross_track_error"),
        "mean_heading_error": number(metrics, "mean_heading_error"),
        "max_heading_error": number(metrics, "max_heading_error"),
        "goal_distance": number(metrics, "goal_distance"),
        "return_code": return_code,
        "output_dir": str(output_dir),
        "error": error,
    }


def dynamic_row(
    metrics: dict,
    return_code: int,
    scenario_name: str,
    map_path: Path,
    controller: str,
    local_planner: str,
    output_dir: Path,
    error: str,
) -> dict:
    return {
        "scenario": scenario_name,
        "map": str(map_path),
        "controller": controller,
        "local_planner": local_planner,
        "success": bool_value(metrics.get("success", False)),
        "goal_reached": bool_value(metrics.get("goal_reached", False)),
        "safe_stop": bool_value(metrics.get("safe_stop", False)),
        "frames_requested": integer(metrics, "frames_requested"),
        "frames_run": integer(metrics, "frames_run"),
        "steps": integer(metrics, "steps"),
        "replanning_count": integer(metrics, "replanning_count"),
        "local_planner_adjustments": integer(
            metrics, "local_planner_adjustments"),
        "local_planner_rollouts": integer(metrics, "local_planner_rollouts"),
        "dynamic_local_collision_rejections": integer(
            metrics, "dynamic_local_collision_rejections"),
        "local_planner_time_ms": number(metrics, "local_planner_time_ms"),
        "moving_obstacle_update_count": integer(
            metrics, "moving_obstacle_update_count"),
        "moving_obstacle_conflict_count": integer(
            metrics, "moving_obstacle_conflict_count"),
        "collision_steps": integer(metrics, "collision_steps"),
        "total_dstar_replanning_time_ms": number(
            metrics, "total_dstar_replanning_time_ms"),
        "total_astar_replanning_time_ms": number(
            metrics, "total_astar_replanning_time_ms"),
        "max_control_jump": number(metrics, "max_control_jump"),
        "mean_control_jump": number(metrics, "mean_control_jump"),
        "minimum_dynamic_obstacle_clearance": number(
            metrics, "minimum_dynamic_obstacle_clearance"),
        "prediction_risk_weight": number(metrics, "prediction_risk_weight"),
        "prediction_risk_clearance": number(
            metrics, "prediction_risk_clearance"),
        "goal_distance": number(metrics, "goal_distance"),
        "return_code": return_code,
        "output_dir": str(output_dir),
        "error": error,
    }


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        path.write_text("")
        return
    fields = list(rows[0])
    if "repeat" in fields:
        fields.remove("repeat")
        fields.insert(0, "repeat")
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def mean(rows: list[dict], key: str) -> float:
    values = [float(row.get(key, 0.0)) for row in rows]
    return statistics.fmean(values) if values else 0.0


def report(rows: list[dict], dynamic: bool = False) -> str:
    if not rows:
        return "No benchmark rows were produced."
    grouped: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for row in rows:
        grouped[(row["controller"], row["local_planner"])].append(row)

    lines = [
        "RobotNav local planner benchmark",
        "mode: dynamic" if dynamic else "mode: static",
        "",
        "controller  local_planner  runs  success  goal_reached  "
        "mean_steps  mean_rollouts  local_ms  mean_clearance",
        "----------  -------------  ----  -------  ------------  "
        "----------  -------------  --------  --------------",
    ]
    for (controller, planner), group in sorted(grouped.items()):
        success = 100.0 * sum(bool_value(row["success"]) for row in group) / len(group)
        reached = 100.0 * sum(
            bool_value(row["goal_reached"]) for row in group) / len(group)
        step_key = "steps" if dynamic else "controller_trace_steps"
        lines.append(
            f"{controller:<10}  {planner:<13}  {len(group):>4}  "
            f"{success:>6.1f}%  {reached:>11.1f}%  "
            f"{mean(group, step_key):>10.1f}  "
            f"{mean(group, 'local_planner_rollouts'):>13.1f}  "
            f"{mean(group, 'local_planner_time_ms'):>8.3f}  "
            f"{mean(group, 'minimum_dynamic_obstacle_clearance'):>14.4f}"
        )
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Benchmark RobotNav DWA/MPPI/local-controller combinations")
    parser.add_argument("--build_dir", default="build")
    parser.add_argument(
        "--data_dir", default=str(AUTOPLANNER_DIR / "data"))
    parser.add_argument(
        "--scenario_file",
        default=str(AUTOPLANNER_DIR / "data/configs/navigation_pipeline.yaml"),
    )
    parser.add_argument(
        "--output_dir", default="autoplanner/results/local_planner_benchmark")
    parser.add_argument("--scenarios", nargs="*", default=list(SCENARIOS),
                        choices=tuple(SCENARIOS))
    parser.add_argument("--local_planners", nargs="*", default=list(LOCAL_PLANNERS),
                        choices=LOCAL_PLANNERS)
    parser.add_argument("--controllers", nargs="*", default=list(CONTROLLERS),
                        choices=CONTROLLERS)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--planner", default="astar")
    parser.add_argument("--dynamic-planner", default="space_time_astar")
    parser.add_argument("--smooth", choices=("none", "shortcut", "curvature"),
                        default="shortcut")
    parser.add_argument("--max-steps", type=int, default=1600)
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--steps-per-frame", type=int, default=40)
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("--skip-dynamic", action="store_true")
    parser.add_argument("--mppi-prediction-time", type=float, default=0.8)
    parser.add_argument("--mppi-horizon", type=int, default=10)
    parser.add_argument("--mppi-rollouts", type=int, default=32)
    parser.add_argument("--mppi-temperature", type=float, default=0.5)
    parser.add_argument("--mppi-velocity-noise", type=float, default=0.35)
    parser.add_argument("--mppi-steering-noise", type=float, default=0.12)
    parser.add_argument("--dwa-prediction-time", type=float, default=0.8)
    parser.add_argument("--dwa-velocity-samples", type=int, default=7)
    parser.add_argument("--dwa-steering-samples", type=int, default=9)
    parser.add_argument("--dwa-dynamic-collision-samples", type=int, default=3)
    parser.add_argument("--moving-obstacle-radius", type=float, default=0.0)
    parser.add_argument(
        "--moving-obstacle-uncertainty-growth", type=float, default=0.0)
    parser.add_argument("--moving-obstacle-acceleration", nargs=2,
                        type=float, default=(0.0, 0.0),
                        metavar=("AX", "AY"))
    parser.add_argument("--moving-obstacle-covariance", nargs=3,
                        type=float, default=(0.0, 0.0, 0.0),
                        metavar=("XX", "XY", "YY"))
    parser.add_argument("--moving-obstacle-covariance-growth", nargs=3,
                        type=float, default=(0.0, 0.0, 0.0),
                        metavar=("XX", "XY", "YY"))
    parser.add_argument("--moving-obstacle-confidence-scale",
                        type=float, default=2.0)
    parser.add_argument("--prediction-risk-weight", type=float, default=0.0)
    parser.add_argument("--prediction-risk-clearance", type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.repeat <= 0:
        print("--repeat must be positive", file=sys.stderr)
        return 2

    build_dir = resolve_path(args.build_dir)
    data_dir = resolve_path(args.data_dir)
    scenario_file = resolve_path(args.scenario_file)
    output_dir = resolve_path(args.output_dir)
    static_cli = build_dir / "apps/navigation_pipeline_cli"
    dynamic_cli = build_dir / "apps/dynamic_navigation_pipeline_cli"
    if not static_cli.exists():
        print(f"Missing executable: {static_cli}", file=sys.stderr)
        return 1
    if not scenario_file.exists():
        print(f"Missing scenario file: {scenario_file}", file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)
    static_rows: list[dict] = []
    dynamic_rows: list[dict] = []

    for repeat in range(args.repeat):
        for scenario_name in args.scenarios:
            scenario = SCENARIOS[scenario_name]
            map_path = data_dir / scenario["map"]
            if not map_path.exists():
                print(f"[SKIP] map not found: {map_path}", file=sys.stderr)
                continue
            for controller in args.controllers:
                for local_planner in args.local_planners:
                    run_name = (
                        f"{scenario_name}_{controller}_{local_planner}_{repeat}")
                    static_output = output_dir / "static" / run_name
                    command = static_command(
                        static_cli, scenario_file, map_path,
                        scenario["start"], scenario["goal"], controller,
                        local_planner, static_output, args)
                    print(f"[static] {run_name}")
                    return_code, error = run_command(command, args.timeout)
                    metrics = read_json(static_output / "metrics.json")
                    row = static_row(
                        metrics, return_code, scenario_name, map_path,
                        controller, local_planner, static_output, error)
                    row["repeat"] = repeat
                    static_rows.append(row)

                    if args.skip_dynamic or not dynamic_cli.exists():
                        continue
                    dynamic_output = output_dir / "dynamic" / run_name
                    command = dynamic_command(
                        dynamic_cli, scenario_file, map_path,
                        scenario["start"], scenario["goal"], controller,
                        local_planner, dynamic_output, args)
                    print(f"[dynamic] {run_name}")
                    return_code, error = run_command(command, args.timeout)
                    metrics = read_json(dynamic_output / "metrics.json")
                    row = dynamic_row(
                        metrics, return_code, scenario_name, map_path,
                        controller, local_planner, dynamic_output, error)
                    row["repeat"] = repeat
                    dynamic_rows.append(row)

    write_csv(output_dir / "local_planner_results.csv", static_rows)
    write_csv(output_dir / "dynamic_local_planner_results.csv", dynamic_rows)
    static_report = report(static_rows)
    dynamic_report = report(dynamic_rows, dynamic=True)
    (output_dir / "local_planner_report.txt").write_text(
        static_report + "\n" + dynamic_report)
    manifest = {
        "planner": args.planner,
        "dynamic_planner": args.dynamic_planner,
        "scenarios": args.scenarios,
        "local_planners": args.local_planners,
        "controllers": args.controllers,
        "repeat": args.repeat,
        "dynamic": not args.skip_dynamic and dynamic_cli.exists(),
        "mppi": {
            "prediction_time": args.mppi_prediction_time,
            "horizon": args.mppi_horizon,
            "rollouts": args.mppi_rollouts,
            "temperature": args.mppi_temperature,
        },
        "dynamic_prediction": {
            "risk_weight": args.prediction_risk_weight,
            "risk_clearance": args.prediction_risk_clearance,
            "moving_obstacle_radius": args.moving_obstacle_radius,
            "moving_obstacle_uncertainty_growth":
                args.moving_obstacle_uncertainty_growth,
            "moving_obstacle_acceleration": args.moving_obstacle_acceleration,
            "moving_obstacle_covariance": args.moving_obstacle_covariance,
            "moving_obstacle_covariance_growth":
                args.moving_obstacle_covariance_growth,
            "moving_obstacle_confidence_scale":
                args.moving_obstacle_confidence_scale,
        },
        "files": [
            "local_planner_results.csv",
            "dynamic_local_planner_results.csv",
            "local_planner_report.txt",
        ],
        "static_rows": len(static_rows),
        "dynamic_rows": len(dynamic_rows),
    }
    (output_dir / "local_planner_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n")
    print("\n" + static_report)
    if dynamic_rows:
        print(dynamic_report)
    print(f"Artifacts: {output_dir}")
    return 0 if static_rows else 2


if __name__ == "__main__":
    raise SystemExit(main())
