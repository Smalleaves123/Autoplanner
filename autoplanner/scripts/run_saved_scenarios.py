#!/usr/bin/env python3
"""Run portable RobotNav dashboard scenarios and write a regression ledger.

With no arguments, runs a bundled dynamic-replanning example. Pass one or
more ``--scene`` YAML files saved by RobotNav Lab to reproduce those edits
without opening the dashboard.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
from pathlib import Path
from typing import Any

from dashboard_support import (
    MovingObstaclePrediction,
    decide_prediction_risk,
    load_grid,
    load_scene,
    write_grid,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
PIPELINE_SCRIPT = SCRIPT_DIR / "run_navigation_pipeline.py"
DEFAULT_MAP = REPO_ROOT / "autoplanner" / "data" / "maps" / "simple_50x50.txt"


def builtin_scene() -> dict[str, Any]:
    """Return a quick, reproducible dynamic scenario for zero-argument runs."""
    return {
        "name": "builtin_dynamic_prediction",
        "grid": load_grid(DEFAULT_MAP),
        "settings": {
            "engine": "dynamic",
            "planner": "improved_astar",
            "controller": "stanley",
            "smooth": "none",
            "velocity": 1.0,
            "robot_radius": 1.0,
            "mpc_horizon": 15,
            "start_x": 1,
            "start_y": 1,
            "goal_x": 20,
            "goal_y": 20,
            "frames": 20,
            "steps_per_frame": 40,
            "obstacle_ahead": 5,
            "auto_obstacles": False,
            "use_prediction": True,
        },
        "prediction": MovingObstaclePrediction(3, 15, 12, 12, 0, 1),
    }


def load_scenario(path: Path) -> dict[str, Any]:
    scene = load_scene(path)
    scene["name"] = path.stem
    return scene


def run_command(scene: dict[str, Any], output_dir: Path,
                build_dir: str, plot: bool) -> tuple[list[str], dict[str, Any]]:
    settings = scene["settings"]
    start = (int(settings["start_x"]), int(settings["start_y"]))
    goal = (int(settings["goal_x"]), int(settings["goal_y"]))
    prediction = scene.get("prediction")
    decision = None
    if prediction is not None and settings.get("engine") == "dynamic":
        decision = decide_prediction_risk(
            prediction, start, goal, float(settings.get("robot_radius", 1.0)))

    velocity_scale = decision.velocity_scale if decision else 1.0
    frames = int(settings.get("frames", 20))
    effective_frames = min(60, math.ceil(frames / max(velocity_scale, 0.1)))
    map_path = output_dir / "scenario_map.txt"
    write_grid(map_path, scene["grid"])
    command = [
        sys.executable, str(PIPELINE_SCRIPT),
        "--build_dir", build_dir,
        "--engine", str(settings.get("engine", "unified")),
        "--map", str(map_path),
        "--planner", str(settings.get("planner", "improved_astar")),
        "--controller", str(settings.get("controller", "stanley")),
        "--start", str(start[0]), str(start[1]),
        "--goal", str(goal[0]), str(goal[1]),
        "--velocity", str(float(settings.get("velocity", 1.0)) * velocity_scale),
        "--robot-radius", str(float(settings.get("robot_radius", 1.0))),
        "--smooth", str(settings.get("smooth", "shortcut")),
        "--mpc-horizon", str(int(settings.get("mpc_horizon", 15))),
        "--output_dir", str(output_dir),
    ]
    if plot:
        command.append("--plot")
    if settings.get("engine") == "dynamic":
        command.extend((
            "--frames", str(effective_frames),
            "--steps-per-frame", str(int(settings.get("steps_per_frame", 40))),
            "--obstacle-ahead", str(int(settings.get("obstacle_ahead", 5))),
        ))
        if not settings.get("auto_obstacles", True):
            command.append("--no-auto-obstacles")
        if decision is not None and decision.action == "replan_slow":
            command.extend(("--moving-obstacle", *(str(value) for value in prediction.cli_values())))
    metadata = {
        "decision": decision.action if decision else "not_applicable",
        "decision_reason": decision.reason if decision else "no dynamic prediction",
        "minimum_clearance": decision.minimum_clearance if decision else None,
        "velocity_scale": velocity_scale,
        "effective_frames": effective_frames,
    }
    return command, metadata


def summary_metrics(output_dir: Path) -> dict[str, Any]:
    path = output_dir / "summary.json"
    if not path.exists():
        return {}
    summary = json.loads(path.read_text())
    metrics = summary.get("pipeline", summary.get("tracking", {}))
    return metrics if isinstance(metrics, dict) else {}


def write_report(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "scenario", "repeat", "return_code", "decision", "velocity_scale",
        "effective_frames", "goal_reached", "safe_stop", "replanning_count",
        "collision_steps", "minimum_clearance", "output_dir",
    )
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene", action="append", default=[], metavar="YAML",
        help="Dashboard YAML scene; repeat for a batch (default: bundled example)",
    )
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--output-dir", default="autoplanner/results/scenario_runs")
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--plot", action="store_true", default=False)
    args = parser.parse_args()
    if args.repeat <= 0:
        parser.error("repeat must be positive")

    output_root = Path(args.output_dir)
    if not output_root.is_absolute():
        output_root = REPO_ROOT / output_root
    scenes = [load_scenario((REPO_ROOT / path).resolve()) for path in args.scene]
    if not scenes:
        scenes = [builtin_scene()]

    rows: list[dict[str, Any]] = []
    for scene in scenes:
        for repeat in range(args.repeat):
            output_dir = output_root / scene["name"] / f"run_{repeat:02d}"
            command, metadata = run_command(scene, output_dir, args.build_dir, args.plot)
            if metadata["decision"] == "safe_stop":
                metrics = {
                    "goal_reached": False,
                    "safe_stop": True,
                    "replanning_count": 0,
                    "collision_steps": 0,
                }
                (output_dir / "summary.json").write_text(json.dumps({
                    "engine": scene["settings"].get("engine", "dynamic"),
                    "pipeline": metrics,
                    "decision": metadata,
                }, indent=2) + "\n")
                completed = subprocess.CompletedProcess(command, 0, "", "")
            else:
                completed = subprocess.run(
                    command, cwd=REPO_ROOT, text=True, capture_output=True)
                metrics = summary_metrics(output_dir)
            row = {
                "scenario": scene["name"],
                "repeat": repeat,
                "return_code": completed.returncode,
                "decision": metadata["decision"],
                "velocity_scale": metadata["velocity_scale"],
                "effective_frames": metadata["effective_frames"],
                "goal_reached": metrics.get("goal_reached", ""),
                "safe_stop": metrics.get("safe_stop", ""),
                "replanning_count": metrics.get("replanning_count", ""),
                "collision_steps": metrics.get("collision_steps", ""),
                "minimum_clearance": metadata["minimum_clearance"],
                "output_dir": str(output_dir),
            }
            rows.append(row)
            status = "ok" if completed.returncode == 0 else f"failed ({completed.returncode})"
            print(f"{scene['name']} run {repeat}: {status}")
            if completed.returncode != 0 and completed.stderr:
                print(completed.stderr, file=sys.stderr)
    report_path = output_root / "scenario_report.csv"
    write_report(report_path, rows)
    print(f"Scenario report: {report_path}")
    return 0 if all(row["return_code"] == 0 for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
