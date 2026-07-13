#!/usr/bin/env python3
"""Run the complete AutoPlanner -> AutoMPC navigation pipeline.

The script keeps the algorithms in C++ and uses Python only to orchestrate
the experiment and combine machine-readable results.

Usage from the repository root:
    python3 autoplanner/scripts/run_navigation_pipeline.py \
        --build_dir build \
        --map autoplanner/data/maps/simple_50x50.txt \
        --planner improved_astar \
        --controller stanley
"""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]


def resolve_path(value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    if path.exists():
        return path.resolve()
    return (REPO_ROOT / path).resolve()


def path_length(path_file: Path) -> float:
    points = []
    with path_file.open() as f:
        next(f, None)
        for line in f:
            x_text, y_text = line.strip().split(",")
            points.append((float(x_text), float(y_text)))

    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(points, points[1:])
    )


def read_json(path: Path) -> dict:
    with path.open() as f:
        return json.load(f)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run AutoPlanner to AutoMPC")
    parser.add_argument("--build_dir", default="build")
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--planner", default="improved_astar")
    parser.add_argument("--controller", default="stanley",
                        choices=("pid", "pure_pursuit", "stanley", "mpc"))
    parser.add_argument("--mpc-horizon", type=int, default=15)
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-steering", type=float, default=0.7)
    parser.add_argument("--start", nargs=2, type=int, default=(1, 1))
    parser.add_argument("--goal", nargs=2, type=int, default=(48, 48))
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--robot-radius", type=float, default=1.0)
    parser.add_argument("--footprint", choices=("point", "circle", "rectangle"),
                        default="point")
    parser.add_argument("--robot-length", type=float, default=0.0)
    parser.add_argument("--robot-width", type=float, default=0.0)
    parser.add_argument("--inflate", action="store_true")
    parser.add_argument("--weight", type=float, default=1.5)
    parser.add_argument("--smooth", choices=("none", "shortcut"), default="shortcut")
    parser.add_argument("--smooth-iterations", type=int, default=100)
    parser.add_argument("--output_dir", default="results/navigation_pipeline")
    args = parser.parse_args()

    build_dir = resolve_path(args.build_dir)
    map_path = resolve_path(args.map)
    output_dir = resolve_path(args.output_dir)
    planning_dir = output_dir / "planning"
    tracking_csv = output_dir / "tracking.csv"
    tracking_metrics = output_dir / "tracking_metrics.json"
    planning_dir.mkdir(parents=True, exist_ok=True)

    planner_cli = build_dir / "apps" / "autoplanner_cli"
    tracker_cli = build_dir / "apps" / "autompc_cli"
    if not planner_cli.exists() or not tracker_cli.exists():
        print("Build executables not found. Build the project first:", file=sys.stderr)
        print("  cmake -S . -B build -DCMAKE_BUILD_TYPE=Release", file=sys.stderr)
        print("  cmake --build build -j", file=sys.stderr)
        return 1

    planner_cmd = [
        str(planner_cli),
        "--planner", args.planner,
        "--map", str(map_path),
        "--start", str(args.start[0]), str(args.start[1]),
        "--goal", str(args.goal[0]), str(args.goal[1]),
        "--output", str(planning_dir),
    ]
    if args.planner == "improved_astar":
        planner_cmd += ["--robot-radius", str(args.robot_radius)]
    if args.planner == "weighted_astar":
        planner_cmd += ["--weight", str(args.weight)]
    if args.footprint != "point":
        planner_cmd += ["--footprint", args.footprint,
                        "--robot-radius", str(args.robot_radius)]
    if args.footprint == "rectangle":
        planner_cmd += ["--robot-length", str(args.robot_length),
                        "--robot-width", str(args.robot_width)]
    if args.inflate:
        planner_cmd.append("--inflate")
    if args.smooth != "none":
        planner_cmd += ["--smooth", args.smooth,
                        "--smooth-iterations", str(args.smooth_iterations)]

    print("[1/2] Planning...")
    planning = subprocess.run(planner_cmd, text=True)
    path_file = planning_dir / "path.csv"
    planning_metrics_file = planning_dir / "metrics.json"
    if planning.returncode != 0 or not path_file.exists() or not planning_metrics_file.exists():
        print("Planning failed; see the planner output above.", file=sys.stderr)
        return planning.returncode or 2

    total_length = path_length(path_file)
    steps = args.steps
    if steps is None:
        distance_per_step = max(args.velocity * args.dt, 1e-6)
        steps = max(100, math.ceil(total_length / distance_per_step) + 100)

    tracker_cmd = [
        str(tracker_cli),
        "--controller", args.controller,
        "--trajectory", "path",
        "--path", str(path_file),
        "--velocity", str(args.velocity),
        "--steps", str(steps),
        "--dt", str(args.dt),
        "--output", str(tracking_csv),
        "--metrics", str(tracking_metrics),
    ]
    if args.controller == "mpc":
        tracker_cmd += ["--mpc-horizon", str(args.mpc_horizon),
                        "--max-velocity", str(args.max_velocity),
                        "--max-steering", str(args.max_steering)]

    print(f"[2/2] Tracking {total_length:.2f} m with {steps} steps...")
    tracking = subprocess.run(tracker_cmd, text=True)
    if tracking.returncode != 0 or not tracking_metrics.exists():
        print("Tracking failed; see the tracker output above.", file=sys.stderr)
        return tracking.returncode or 3

    summary = {
        "planner": read_json(planning_metrics_file),
        "tracking": read_json(tracking_metrics),
        "path_length_from_csv": total_length,
        "tracking_steps": steps,
    }
    summary_file = output_dir / "summary.json"
    with summary_file.open("w") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")

    print(f"Pipeline complete: {output_dir}")
    print(f"Summary: {summary_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
