#!/usr/bin/env python3
"""Run a deterministic tracking benchmark across kinematic execution models.

The planner-independent polyline, controller configuration, seed, and route
are identical for both runs. Each execution is written as a versioned
``RunArtifact`` and the two artifacts share one ``ExperimentManifest`` so
results can be compared without simulator-specific readers.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import robotnav

from experiment_schema import (  # type: ignore
    BackendSpec,
    ExperimentManifest,
    RunArtifact,
    RunMetrics,
    ScenarioSpec,
    SimulationSpec,
)


DEFAULT_WAYPOINTS = (
    (0.0, 0.0), (3.0, 0.0), (6.0, 1.5), (9.0, 1.5),
)


def path_length(states: Iterable[robotnav.RobotState],
                initial: robotnav.RobotState) -> float:
    previous = initial
    length = 0.0
    for state in states:
        length += math.hypot(state.x - previous.x, state.y - previous.y)
        previous = state
    return length


def percentile(values: Iterable[float], quantile: float) -> float | None:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    index = quantile * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    fraction = index - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


def write_trace(
        path: Path,
        states: tuple[robotnav.RobotState, ...],
        controls: tuple[robotnav.ControlCommand, ...],
        config: robotnav.SimulationConfig,
        compute_latencies: tuple[float, ...]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = (
        "step", "time_s", "x", "y", "theta", "velocity",
        "command_velocity", "command_steering", "twist_linear_velocity",
        "twist_angular_velocity", "compute_latency_ms",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, (state, command) in enumerate(zip(states, controls), 1):
            twist = robotnav.steering_to_twist(command, config.wheelbase)
            writer.writerow({
                "step": index,
                "time_s": index * config.dt,
                "x": state.x,
                "y": state.y,
                "theta": state.theta,
                "velocity": state.velocity,
                "command_velocity": command.velocity,
                "command_steering": command.steering,
                "twist_linear_velocity": twist.linear_velocity,
                "twist_angular_velocity": twist.angular_velocity,
                "compute_latency_ms": compute_latencies[index - 1],
            })


def simulation_spec(config: robotnav.SimulationConfig) -> SimulationSpec:
    return SimulationSpec(
        dt=config.dt,
        wheelbase=config.wheelbase,
        max_velocity=config.max_velocity,
        max_acceleration=config.max_acceleration,
        max_deceleration=config.max_deceleration,
        max_steering=config.max_steering,
        max_steering_rate=config.max_steering_rate,
        track_width=config.track_width,
        max_angular_velocity=config.max_angular_velocity,
        max_angular_acceleration=config.max_angular_acceleration,
        max_wheel_velocity=config.max_wheel_velocity,
    )


def run_model(
        model: str,
        trajectory: robotnav.TrajectoryResult,
        initial: robotnav.RobotState,
        args: argparse.Namespace,
        output_dir: Path,
) -> RunArtifact:
    config = robotnav.SimulationConfig(
        dt=args.dt,
        wheelbase=args.wheelbase,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        max_deceleration=args.max_deceleration,
        max_steering=args.max_steering,
        max_steering_rate=args.max_steering_rate,
        execution_model=model,
        track_width=args.track_width,
        max_angular_velocity=args.max_angular_velocity,
        max_angular_acceleration=args.max_angular_acceleration,
        max_wheel_velocity=args.max_wheel_velocity,
    )
    controller = robotnav.Controller(robotnav.ControllerConfig(
        controller=args.controller,
        target_velocity=args.velocity,
        dt=args.dt,
        wheelbase=args.wheelbase,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
        max_deceleration=args.max_deceleration,
        max_steering=args.max_steering,
        max_steering_rate=args.max_steering_rate,
    ))
    result = robotnav.simulate(
        initial, trajectory, controller, config=config,
        max_time=args.max_time)
    goal = trajectory.points[-1]
    final = result.states[-1] if result.states else initial
    goal_distance = math.hypot(final.x - goal.x, final.y - goal.y)
    goal_reached = goal_distance <= args.goal_tolerance
    steps = len(result.states)
    actual_path = path_length(result.states, initial)
    control_effort = 0.0
    for command in result.controls:
        twist = robotnav.steering_to_twist(command, config.wheelbase)
        control_effort += (
            twist.linear_velocity ** 2 + twist.angular_velocity ** 2
        ) * config.dt

    trace_path = output_dir / f"trace_{model}.csv"
    write_trace(
        trace_path, result.states, result.controls, config,
        result.compute_latency_ms)
    backend_model = {
        "kinematic_bicycle": "constrained_bicycle",
        "differential_drive": "constrained_differential_drive",
    }[model]
    scenario = ScenarioSpec(
        scenario_id=args.scenario_id,
        backend=BackendSpec("kinematic", backend_model),
        simulation=simulation_spec(config),
        map_path="",
        path="inline://cross_model_default",
        start=(initial.x, initial.y), goal=(goal.x, goal.y),
        planner="none", controller=args.controller,
        max_steps=max(1, math.ceil(args.max_time / args.dt)),
        goal_tolerance=args.goal_tolerance,
        seed=args.seed,
        metadata={
            "benchmark": "cross_model_tracking",
            "execution_model": model,
            "command_contract": "steering_to_twist",
        },
    )
    metrics = RunMetrics(
        status="goal_reached" if goal_reached else "step_limit",
        run_success=True,
        goal_reached=goal_reached,
        steps=steps,
        goal_time_s=steps * config.dt if goal_reached else None,
        goal_distance=goal_distance,
        actual_path_length=actual_path,
        max_cross_track=result.metrics.max_cross_track,
        mean_cross_track=result.metrics.mean_cross_track,
        max_heading_error=result.metrics.max_heading_error,
        mean_heading_error=result.metrics.mean_heading_error,
        control_effort=control_effort,
        compute_latency_ms={
            "p50": percentile(result.compute_latency_ms, 0.50),
            "p95": percentile(result.compute_latency_ms, 0.95),
            "p99": percentile(result.compute_latency_ms, 0.99),
        },
    )
    return RunArtifact(
        scenario=scenario,
        metrics=metrics,
        trace_csv=str(trace_path),
        metadata={"execution_model": model},
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", default="autoplanner/results/cross_model")
    parser.add_argument("--scenario-id", default="cross_model_tracking")
    parser.add_argument("--model", choices=(
        "both", "kinematic_bicycle", "differential_drive"), default="both")
    parser.add_argument("--controller", choices=("stanley", "mpc"),
                        default="stanley")
    parser.add_argument("--velocity", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--max-time", type=float, default=12.0)
    parser.add_argument("--goal-tolerance", type=float, default=0.75)
    parser.add_argument("--wheelbase", type=float, default=1.0)
    parser.add_argument("--track-width", type=float, default=0.55)
    parser.add_argument("--max-velocity", type=float, default=2.0)
    parser.add_argument("--max-acceleration", type=float, default=1.5)
    parser.add_argument("--max-deceleration", type=float, default=2.0)
    parser.add_argument("--max-steering", type=float, default=0.7)
    parser.add_argument("--max-steering-rate", type=float, default=1.5)
    parser.add_argument("--max-angular-velocity", type=float, default=3.0)
    parser.add_argument("--max-angular-acceleration", type=float, default=4.0)
    parser.add_argument("--max-wheel-velocity", type=float, default=2.5)
    parser.add_argument("--seed", type=int, default=0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.seed < 0:
        raise SystemExit("--seed must be non-negative")
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
    models = ("kinematic_bicycle", "differential_drive") \
        if args.model == "both" else (args.model,)
    runs = tuple(run_model(model, trajectory, initial, args, output_dir)
                 for model in models)
    manifest = ExperimentManifest(args.scenario_id, runs)
    manifest_path = output_dir / "experiment_manifest.json"
    manifest.save_json(manifest_path)
    print(manifest_path)
    for run in runs:
        print(json_summary(run))
    return 0


def json_summary(run: RunArtifact) -> str:
    return (
        f"{run.scenario.backend.model}: status={run.metrics.status} "
        f"goal_distance={run.metrics.goal_distance:.3f} "
        f"steps={run.metrics.steps}")


if __name__ == "__main__":
    raise SystemExit(main())
