#!/usr/bin/env python3
"""Closed-loop planner/controller/execution-backend benchmark.

The C++ planner produces a waypoint path, the C++ trajectory layer produces a
curvature-aware reference, and a C++ Stanley or MPC controller is evaluated
against the C++ kinematic simulator, MuJoCo, or PyBullet. Raw per-step CSV and
versioned JSON files are written using one backend-neutral schema.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

from physics_backend_smoke import (  # type: ignore
    MujocoBicycleSimulator,
    PhysicsOptions,
    PyBulletBicycleSimulator,
    PyBulletRacecarSimulator,
)
from experiment_schema import (  # type: ignore
    BackendSpec,
    ExperimentManifest,
    RunArtifact,
    RunMetrics,
    ScenarioSpec,
    SimulationSpec,
)
from validation_suites import PerturbationSpec, PerturbedSimulator  # type: ignore


class KinematicBicycleSimulator:
    """Adapt the C++ kinematic bicycle to the physics simulator protocol."""

    def __init__(self, options: PhysicsOptions, autompc_module: Any | None = None):
        if autompc_module is None:
            import autompc as autompc_module
        self.autompc = autompc_module
        native_options = autompc_module.SimulationOptions()
        for name in (
                "dt", "wheelbase", "max_velocity", "max_acceleration",
                "max_deceleration", "max_steering", "max_steering_rate"):
            setattr(native_options, name, getattr(options, name))
        self.simulator = autompc_module.KinematicBicycleSimulator(
            autompc_module.State(0.0, 0.0, 0.0, 0.0), native_options)

    def reset(self, x: float = 0.0, y: float = 0.0,
              theta: float = 0.0, velocity: float = 0.0) -> None:
        self.simulator.reset(self.autompc.State(x, y, theta, velocity))

    def observe(self) -> dict[str, float]:
        state = self.simulator.state
        return {
            "x": float(state.x), "y": float(state.y),
            "theta": float(state.theta), "v": float(state.v),
            "contacts": 0.0, "obstacle_contacts": 0.0,
        }

    def step(self, velocity: float, steering: float) -> dict[str, float]:
        self.simulator.step(self.autompc.Control(velocity, steering))
        return self.observe()


def backend_model(backend_name: str, pybullet_model: str) -> str:
    if backend_name == "kinematic":
        return "constrained_bicycle"
    if backend_name == "mujoco":
        return "planar"
    return pybullet_model


def create_simulator(backend_name: str, options: PhysicsOptions,
                     pybullet_model: str, autompc_module: Any) -> Any:
    """Construct a simulator implementing reset/observe/step/close(optional)."""

    if backend_name == "kinematic":
        return KinematicBicycleSimulator(options, autompc_module)
    if backend_name == "mujoco":
        return MujocoBicycleSimulator(options)
    if pybullet_model == "racecar":
        return PyBulletRacecarSimulator(options)
    return PyBulletBicycleSimulator(options)


def perturbation_from_args(args: argparse.Namespace) -> PerturbationSpec:
    return PerturbationSpec(
        position_noise_std=args.position_noise_std,
        heading_noise_std=args.heading_noise_std,
        velocity_noise_std=args.velocity_noise_std,
        observation_latency_steps=args.observation_latency_steps,
        command_velocity_limit=args.command_velocity_limit,
        command_steering_limit=args.command_steering_limit,
    )


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def percentile(values: list[float], quantile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = quantile * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    fraction = index - lower
    return ordered[lower] + fraction * (ordered[upper] - ordered[lower])


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
        planner_metrics: dict[str, Any], root: Path) -> RunArtifact:
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
    simulator = PerturbedSimulator(
        create_simulator(
            backend_name, physics_options, args.pybullet_model, autompc),
        perturbation_from_args(args), args.seed)
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
        "step", "x", "y", "theta", "v", "observed_x", "observed_y",
        "observed_theta", "observed_v", "ref_x", "ref_y", "ref_theta",
        "ref_v", "command_velocity", "command_steering",
        "applied_velocity", "applied_steering", "cross_track",
        "heading_error", "goal_distance", "reference_index",
        "obstacle_contacts", "compute_latency_ms",
    ]
    rows: list[dict[str, float | int]] = []
    actual_path_length = 0.0
    controller_state = simulator.observe()
    previous_state = simulator.last_truth
    max_cross_track = 0.0
    max_heading_error = 0.0
    collision_steps = 0
    goal_reached = False
    control_effort = 0.0
    compute_latencies: list[float] = []
    try:
        for step in range(args.steps):
            compute_begin = time.perf_counter()
            current = controller_state
            reference_index, reference = nearest_reference(trajectory, current)
            state_object = autompc.State(
                current["x"], current["y"], current["theta"], current["v"])
            if args.controller == "stanley":
                command = controller.compute(
                    state_object, reference, reference.v)
            else:
                command = controller.compute(
                    state_object, trajectory, reference.v)
            compute_latency_ms = (
                time.perf_counter() - compute_begin) * 1000.0
            compute_latencies.append(compute_latency_ms)
            controller_state = simulator.step(
                command.velocity, command.steering)
            next_state = simulator.last_truth
            applied_velocity = simulator.last_applied_velocity
            applied_steering = simulator.last_applied_steering
            control_effort += args.dt * (
                applied_velocity ** 2 + applied_steering ** 2)
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
                "observed_x": controller_state["x"],
                "observed_y": controller_state["y"],
                "observed_theta": controller_state["theta"],
                "observed_v": controller_state["v"],
                "ref_x": reference.x, "ref_y": reference.y,
                "ref_theta": reference.theta, "ref_v": reference.v,
                "command_velocity": command.velocity,
                "command_steering": command.steering,
                "applied_velocity": applied_velocity,
                "applied_steering": applied_steering,
                "cross_track": cross_track, "heading_error": heading_error,
                "goal_distance": goal_distance,
                "reference_index": reference_index,
                "obstacle_contacts": obstacle_contacts,
                "compute_latency_ms": compute_latency_ms,
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
    scenario = ScenarioSpec(
        scenario_id=args.scenario_id,
        backend=BackendSpec(
            backend_name, backend_model(backend_name, args.pybullet_model)),
        simulation=SimulationSpec(
            dt=args.dt, wheelbase=wheelbase,
            max_velocity=args.max_velocity,
            max_acceleration=args.max_acceleration,
            max_deceleration=args.max_deceleration,
            max_steering=args.max_steering,
            max_steering_rate=args.max_steering_rate),
        map_path=str((root / args.map).resolve()),
        path=str(path),
        start=(float(trajectory[0].x), float(trajectory[0].y)),
        goal=(float(trajectory[-1].x), float(trajectory[-1].y)),
        planner=args.planner,
        controller=args.controller,
        max_steps=args.steps,
        goal_tolerance=args.goal_tolerance,
        initial_offset=args.initial_offset,
        seed=args.seed,
        metadata={
            "sample_spacing": args.sample_spacing,
            "target_velocity": args.velocity,
            "max_lateral_acceleration": args.max_lateral_acceleration,
            "perturbation": perturbation_from_args(args).to_dict(),
        },
    )
    metrics = RunMetrics(
        status="goal_reached" if goal_reached else "step_limit",
        run_success=True,
        goal_reached=goal_reached,
        steps=len(rows),
        goal_time_s=len(rows) * args.dt if goal_reached else None,
        goal_distance=float(final.get("goal_distance", 0.0)),
        actual_path_length=actual_path_length,
        max_cross_track=max_cross_track,
        mean_cross_track=(
            sum(float(row["cross_track"]) for row in rows) / len(rows)
            if rows else 0.0),
        max_heading_error=max_heading_error,
        mean_heading_error=(
            sum(float(row["heading_error"]) for row in rows) / len(rows)
            if rows else 0.0),
        collision_steps=collision_steps,
        control_effort=control_effort,
        compute_latency_ms={
            "p50": percentile(compute_latencies, 0.50),
            "p95": percentile(compute_latencies, 0.95),
            "p99": percentile(compute_latencies, 0.99),
        },
    )
    artifact = RunArtifact(
        scenario=scenario,
        metrics=metrics,
        trace_csv=str(csv_path),
        planner_metrics=planner_metrics,
    )
    artifact.save_json(json_path)
    return artifact


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--backend",
        choices=("kinematic", "mujoco", "pybullet", "both", "all"),
        default="both",
        help="execution backend; 'both' retains the MuJoCo+PyBullet alias")
    parser.add_argument("--pybullet-model", choices=("racecar", "planar"),
                        default="racecar")
    parser.add_argument("--controller", choices=("stanley", "mpc"),
                        default="stanley")
    parser.add_argument("--path", default=None)
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
    parser.add_argument("--scenario-id", default="physics_tracking")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--position-noise-std", type=float, default=0.0)
    parser.add_argument("--heading-noise-std", type=float, default=0.0)
    parser.add_argument("--velocity-noise-std", type=float, default=0.0)
    parser.add_argument("--observation-latency-steps", type=int, default=0)
    parser.add_argument("--command-velocity-limit", type=float, default=None)
    parser.add_argument("--command-steering-limit", type=float, default=None)
    args = parser.parse_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.seed < 0:
        parser.error("--seed must be non-negative")
    try:
        perturbation_from_args(args)
    except ValueError as error:
        parser.error(str(error))

    root = Path(__file__).resolve().parents[2]
    path, planner_metrics = planner_path(args, root)
    if args.backend == "all":
        backends = ("kinematic", "mujoco", "pybullet")
    elif args.backend == "both":
        backends = ("mujoco", "pybullet")
    else:
        backends = (args.backend,)
    artifacts = []
    for backend in backends:
        artifact = run(args, backend, path, planner_metrics, root)
        artifacts.append(artifact)
        print(json.dumps(artifact.to_dict(), indent=2))
    output_dir = (root / args.output_dir).resolve()
    manifest = ExperimentManifest(args.scenario_id, tuple(artifacts))
    manifest.save_json(output_dir / "experiment_manifest.json")
    print(f"Manifest: {output_dir / 'experiment_manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
