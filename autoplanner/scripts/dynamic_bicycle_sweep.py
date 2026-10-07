#!/usr/bin/env python3
"""Sweep dynamic-bicycle parameters with the deterministic tracking harness."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from types import SimpleNamespace

import robotnav

from cross_model_tracking_benchmark import (  # type: ignore
    DEFAULT_WAYPOINTS,
    run_model,
)


def values(text: str) -> tuple[float, ...]:
    result = tuple(float(item.strip()) for item in text.split(",") if item.strip())
    if not result or any(value <= 0.0 for value in result):
        raise argparse.ArgumentTypeError("values must be positive comma-separated numbers")
    return result


def build_args(args: argparse.Namespace, mass: float, yaw_inertia: float,
               stiffness: float, scenario_id: str) -> argparse.Namespace:
    return SimpleNamespace(
        dt=args.dt, wheelbase=args.wheelbase, max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        max_deceleration=args.max_deceleration,
        max_steering=args.max_steering,
        max_steering_rate=args.max_steering_rate,
        front_axle=args.front_axle, rear_axle=args.rear_axle,
        mass=mass, yaw_inertia=yaw_inertia,
        front_cornering_stiffness=stiffness,
        rear_cornering_stiffness=stiffness,
        track_width=0.55, max_angular_velocity=3.0,
        max_angular_acceleration=4.0, max_wheel_velocity=2.5,
        controller=args.controller, velocity=args.velocity,
        max_time=args.max_time, goal_tolerance=args.goal_tolerance,
        seed=args.seed, scenario_id=scenario_id,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="autoplanner/results/dynamic_sweep")
    parser.add_argument("--masses", type=values, default=(15.0, 20.0, 30.0))
    parser.add_argument("--yaw-inertias", type=values, default=(8.0, 10.0, 15.0))
    parser.add_argument("--stiffnesses", type=values, default=(50.0, 80.0, 120.0))
    parser.add_argument("--controller", choices=("stanley", "mpc"), default="stanley")
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--wheelbase", type=float, default=1.0)
    parser.add_argument("--front-axle", type=float, default=0.45)
    parser.add_argument("--rear-axle", type=float, default=0.55)
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-acceleration", type=float, default=1.5)
    parser.add_argument("--max-deceleration", type=float, default=2.0)
    parser.add_argument("--max-steering", type=float, default=0.7)
    parser.add_argument("--max-steering-rate", type=float, default=1.5)
    parser.add_argument("--max-time", type=float, default=12.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.75)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectory = robotnav.generate_trajectory(
        DEFAULT_WAYPOINTS,
        robotnav.TrajectoryConfig(
            sample_spacing=0.25, target_velocity=args.velocity,
            max_velocity=args.max_velocity,
            max_acceleration=args.max_acceleration,
            max_deceleration=args.max_deceleration),
    )
    initial = robotnav.RobotState(*DEFAULT_WAYPOINTS[0])
    rows = []
    for mass in args.masses:
        for yaw_inertia in args.yaw_inertias:
            for stiffness in args.stiffnesses:
                case = f"mass_{mass:g}_yaw_{yaw_inertia:g}_stiff_{stiffness:g}"
                case_dir = output_dir / case
                run = run_model(
                    "dynamic_bicycle", trajectory, initial,
                    build_args(args, mass, yaw_inertia, stiffness, case),
                    case_dir)
                metrics = run.metrics
                rows.append({
                    "case": case, "mass": mass,
                    "yaw_inertia": yaw_inertia, "stiffness": stiffness,
                    "status": metrics.status,
                    "goal_reached": metrics.goal_reached,
                    "steps": metrics.steps,
                    "goal_distance": metrics.goal_distance,
                    "max_cross_track": metrics.max_cross_track,
                    "mean_cross_track": metrics.mean_cross_track,
                    "control_effort": metrics.control_effort,
                    "latency_p95_ms": metrics.compute_latency_ms.get("p95"),
                })
    with (output_dir / "sweep.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output_dir / "sweep.json").open("w", encoding="utf-8") as stream:
        json.dump({"benchmark": "dynamic_bicycle_sweep", "runs": rows},
                  stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(output_dir / "sweep.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
