#!/usr/bin/env python3
"""Run the complete AutoPlanner -> AutoMPC navigation pipeline.

The script keeps the algorithms in C++ and uses Python only to orchestrate
the experiment and combine machine-readable results.

Usage from the repository root:
    python autoplanner/scripts/run_navigation_pipeline.py \
        --build_dir build \
        --map autoplanner/data/maps/simple_50x50.txt \
        --planner improved_astar --controller stanley --plot
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


def plot_output_path(args, output_dir: Path) -> Path:
    if not args.plot_output:
        return output_dir / "navigation.png"
    path = Path(args.plot_output).expanduser()
    return path if path.is_absolute() else output_dir / path


def render_plot(args, map_path: Path, trace_file: Path, metrics_file: Path,
                output_dir: Path, title: str,
                planned_path: Path | None = None) -> Path | None:
    plotter = SCRIPT_DIR / "visualize_navigation_trace.py"
    if not plotter.exists():
        print(f"Visualization script not found: {plotter}", file=sys.stderr)
        return None

    output = plot_output_path(args, output_dir)
    command = [
        sys.executable,
        str(plotter),
        "--map", str(map_path),
        "--trace", str(trace_file),
        "--metrics", str(metrics_file),
        "--output", str(output),
        "--title", title,
    ]
    if planned_path is not None and planned_path.exists():
        command += ["--path", str(planned_path)]
    print("Rendering navigation plot...")
    completed = subprocess.run(command, text=True)
    if completed.returncode != 0 or not output.exists():
        print("Visualization failed; see the Python output above.",
              file=sys.stderr)
        return None
    return output


def run_unified_pipeline(args, build_dir: Path, map_path: Path,
                         output_dir: Path) -> int:
    """Run the reusable C++ pipeline while preserving the legacy runner."""
    pipeline_cli = build_dir / "apps" / "navigation_pipeline_cli"
    if not pipeline_cli.exists():
        print("Unified pipeline executable not found. Build the project first:",
              file=sys.stderr)
        return 1
    output_dir.mkdir(parents=True, exist_ok=True)
    max_steps = args.steps if args.steps is not None else 2500
    command = [
        str(pipeline_cli),
        "--map", str(map_path),
        "--planner", args.planner,
        "--controller", args.controller,
        "--start", str(args.start[0]), str(args.start[1]),
        "--goal", str(args.goal[0]), str(args.goal[1]),
        "--max-steps", str(max_steps),
        "--velocity", str(args.velocity),
        "--dt", str(args.dt),
        "--footprint", args.footprint,
        "--robot-radius", str(args.robot_radius),
        "--robot-length", str(args.robot_length),
        "--robot-width", str(args.robot_width),
        "--smooth", args.smooth,
        "--smooth-iterations", str(args.smooth_iterations),
        "--output-dir", str(output_dir),
    ]
    if args.inflate:
        command.append("--inflate")
    print("Running unified C++ navigation pipeline...")
    completed = subprocess.run(command, text=True)
    metrics_file = output_dir / "metrics.json"
    trace_file = output_dir / "trace.csv"
    if completed.returncode != 0 or not metrics_file.exists() or not trace_file.exists():
        print("Unified pipeline failed; see the C++ output above.", file=sys.stderr)
        return completed.returncode or 2

    plot_file = None
    if args.plot:
        plot_file = render_plot(
            args, map_path, trace_file, metrics_file, output_dir,
            "RobotNav Unified Navigation")
        if plot_file is None:
            return 4

    summary_file = output_dir / "summary.json"
    summary = {
        "engine": "unified",
        "pipeline": read_json(metrics_file),
        "trace_csv": str(trace_file),
    }
    if plot_file is not None:
        summary["plot_png"] = str(plot_file)
    with summary_file.open("w") as stream:
        json.dump(summary, stream, indent=2)
        stream.write("\n")
    print(f"Pipeline complete: {output_dir}")
    print(f"Summary: {summary_file}")
    return 0


def run_dynamic_pipeline(args, build_dir: Path, map_path: Path,
                         output_dir: Path) -> int:
    """Run the dynamic C++ pipeline and optionally render its trace."""
    pipeline_cli = build_dir / "apps" / "dynamic_navigation_pipeline_cli"
    if not pipeline_cli.exists():
        print("Dynamic pipeline executable not found. Build the project first:",
              file=sys.stderr)
        return 1

    output_dir.mkdir(parents=True, exist_ok=True)
    command = [
        str(pipeline_cli),
        "--map", str(map_path),
        "--planner", args.planner,
        "--controller", args.controller,
        "--start", str(args.start[0]), str(args.start[1]),
        "--goal", str(args.goal[0]), str(args.goal[1]),
        "--footprint", args.footprint,
        "--robot-radius", str(args.robot_radius),
        "--robot-length", str(args.robot_length),
        "--robot-width", str(args.robot_width),
        "--smooth", args.smooth,
        "--velocity", str(args.velocity),
        "--frames", str(args.frames),
        "--steps-per-frame", str(args.steps_per_frame),
        "--obstacle-ahead", str(args.obstacle_ahead),
        "--obstacle-margin", str(args.obstacle_margin),
        "--max-auto-obstacles", str(args.max_auto_obstacles),
        "--output-dir", str(output_dir),
    ]
    if args.inflate:
        command.append("--inflate")
    if args.no_diagonal:
        command.append("--no-diagonal")
    if args.no_auto_obstacles:
        command.append("--no-auto-obstacles")
    for frame, x, y in args.obstacle:
        command += ["--obstacle", str(frame), str(x), str(y)]
    for frame, x, y in args.clear_obstacle:
        command += ["--clear-obstacle", str(frame), str(x), str(y)]
    for start, end, x, y, dx, dy in args.moving_obstacle:
        command += [
            "--moving-obstacle", str(start), str(end), str(x), str(y),
            str(dx), str(dy),
        ]
    if args.moving_obstacle_radius > 0.0:
        command += ["--moving-obstacle-radius",
                    str(args.moving_obstacle_radius)]
    if args.moving_obstacle_uncertainty_growth > 0.0:
        command += ["--moving-obstacle-uncertainty-growth",
                    str(args.moving_obstacle_uncertainty_growth)]
    if args.prediction_risk_weight > 0.0:
        command += ["--prediction-risk-weight",
                    str(args.prediction_risk_weight)]
    if args.prediction_risk_clearance > 0.0:
        command += ["--prediction-risk-clearance",
                    str(args.prediction_risk_clearance)]

    print("Running dynamic C++ navigation pipeline...")
    completed = subprocess.run(command, text=True)
    metrics_file = output_dir / "metrics.json"
    trace_file = output_dir / "trace.csv"
    if completed.returncode != 0 or not metrics_file.exists() or not trace_file.exists():
        print("Dynamic pipeline failed; see the C++ output above.",
              file=sys.stderr)
        return completed.returncode or 2

    plot_file = None
    if args.plot:
        plot_file = render_plot(
            args, map_path, trace_file, metrics_file, output_dir,
            "RobotNav Dynamic Navigation")
        if plot_file is None:
            return 4

    summary = {
        "engine": "dynamic",
        "pipeline": read_json(metrics_file),
        "trace_csv": str(trace_file),
    }
    if plot_file is not None:
        summary["plot_png"] = str(plot_file)
    summary_file = output_dir / "summary.json"
    with summary_file.open("w") as stream:
        json.dump(summary, stream, indent=2)
        stream.write("\n")
    print(f"Pipeline complete: {output_dir}")
    print(f"Summary: {summary_file}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Run AutoPlanner to AutoMPC")
    parser.add_argument("--build_dir", default="build")
    parser.add_argument("--engine", choices=("legacy", "unified", "dynamic"),
                        default="legacy")
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--planner", default="improved_astar")
    parser.add_argument("--controller", default="stanley",
                        choices=("pid", "pure_pursuit", "stanley", "mpc"))
    parser.add_argument("--mpc-horizon", type=int, default=15)
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-steering", type=float, default=0.7)
    parser.add_argument("--max-acceleration", type=float, default=1.5)
    parser.add_argument("--max-deceleration", type=float, default=2.0)
    parser.add_argument("--max-steering-rate", type=float, default=1.5)
    parser.add_argument("--sample-spacing", type=float, default=0.5)
    parser.add_argument("--max-lateral-acceleration", type=float, default=1.5)
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
    parser.add_argument("--inflate", action="store_true", default=False)
    parser.add_argument("--no-diagonal", action="store_true", default=False,
                        help="use 4-connected graph search where supported")
    parser.add_argument("--weight", type=float, default=1.5)
    parser.add_argument("--smooth", choices=("none", "shortcut"), default="shortcut")
    parser.add_argument("--smooth-iterations", type=int, default=100)
    parser.add_argument("--frames", type=int, default=20,
                        help="dynamic pipeline update frames")
    parser.add_argument("--steps-per-frame", type=int, default=40,
                        help="dynamic pipeline control cycles per frame")
    parser.add_argument("--obstacle-ahead", type=int, default=5,
                        help="dynamic auto-obstacle path samples ahead")
    parser.add_argument("--obstacle-margin", type=int, default=1,
                        help="dynamic auto-obstacle safety margin in cells")
    parser.add_argument("--max-auto-obstacles", type=int, default=1)
    parser.add_argument("--no-auto-obstacles", action="store_true", default=False)
    parser.add_argument("--obstacle", nargs=3, type=int, action="append",
                        default=[], metavar=("FRAME", "X", "Y"))
    parser.add_argument("--clear-obstacle", nargs=3, type=int, action="append",
                        default=[], metavar=("FRAME", "X", "Y"))
    parser.add_argument("--moving-obstacle", nargs=6, type=int, action="append",
                        default=[], metavar=("START", "END", "X", "Y", "DX", "DY"))
    parser.add_argument("--moving-obstacle-radius", type=float, default=0.0)
    parser.add_argument("--moving-obstacle-uncertainty-growth",
                        type=float, default=0.0)
    parser.add_argument("--prediction-risk-weight", type=float, default=0.0)
    parser.add_argument("--prediction-risk-clearance", type=float, default=0.0)
    parser.add_argument("--plot", action="store_true", default=False,
                        help="write a navigation.png visual summary")
    parser.add_argument("--plot-output",
                        help="PNG path relative to --output_dir, or absolute")
    parser.add_argument("--output_dir", default="results/navigation_pipeline")
    args = parser.parse_args()

    build_dir = resolve_path(args.build_dir)
    map_path = resolve_path(args.map)
    output_dir = resolve_path(args.output_dir)
    if args.engine == "unified":
        return run_unified_pipeline(args, build_dir, map_path, output_dir)
    if args.engine == "dynamic":
        return run_dynamic_pipeline(args, build_dir, map_path, output_dir)

    planning_dir = output_dir / "planning"
    tracking_csv = output_dir / "tracking.csv"
    tracking_metrics = output_dir / "tracking_metrics.json"
    trajectory_csv = output_dir / "trajectory.csv"
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
    if args.no_diagonal:
        planner_cmd.append("--no-diagonal")
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
        steps = max(100, math.ceil(1.5 * total_length / distance_per_step) + 100)

    tracker_cmd = [
        str(tracker_cli),
        "--controller", args.controller,
        "--trajectory", "path",
        "--path", str(path_file),
        "--velocity", str(args.velocity),
        "--max-velocity", str(args.max_velocity),
        "--max-acceleration", str(args.max_acceleration),
        "--max-deceleration", str(args.max_deceleration),
        "--sample-spacing", str(args.sample_spacing),
        "--max-lateral-acceleration", str(args.max_lateral_acceleration),
        "--steps", str(steps),
        "--dt", str(args.dt),
        "--output", str(tracking_csv),
        "--metrics", str(tracking_metrics),
        "--trajectory-output", str(trajectory_csv),
    ]
    if args.controller == "mpc":
        tracker_cmd += ["--mpc-horizon", str(args.mpc_horizon),
                        "--max-velocity", str(args.max_velocity),
                        "--max-steering", str(args.max_steering),
                        "--max-acceleration", str(args.max_acceleration),
                        "--max-deceleration", str(args.max_deceleration),
                        "--max-steering-rate", str(args.max_steering_rate)]

    print(f"[2/2] Tracking {total_length:.2f} m with {steps} steps...")
    tracking = subprocess.run(tracker_cmd, text=True)
    if (tracking.returncode != 0 or not tracking_metrics.exists() or
            not trajectory_csv.exists()):
        print("Tracking failed; see the tracker output above.", file=sys.stderr)
        return tracking.returncode or 3

    plot_file = None
    if args.plot:
        plot_file = render_plot(
            args, map_path, tracking_csv, tracking_metrics, output_dir,
            "RobotNav Legacy Navigation", path_file)
        if plot_file is None:
            return 4

    summary = {
        "planner": read_json(planning_metrics_file),
        "tracking": read_json(tracking_metrics),
        "path_length_from_csv": total_length,
        "tracking_steps": steps,
        "trajectory_csv": str(trajectory_csv),
    }
    if plot_file is not None:
        summary["plot_png"] = str(plot_file)
    summary_file = output_dir / "summary.json"
    with summary_file.open("w") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")

    print(f"Pipeline complete: {output_dir}")
    print(f"Summary: {summary_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
