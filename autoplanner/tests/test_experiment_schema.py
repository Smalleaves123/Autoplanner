#!/usr/bin/env python3
"""Regression tests for the unified execution experiment schema."""

from __future__ import annotations

import tempfile
import types
import unittest
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from experiment_schema import (  # noqa: E402
    BackendSpec,
    ExperimentManifest,
    RunArtifact,
    RunMetrics,
    ScenarioSpec,
    SimulationSpec,
    artifact_from_legacy_summary,
)
from physics_backend_smoke import PhysicsOptions  # noqa: E402
from physics_tracking_benchmark import (  # noqa: E402
    KinematicBicycleSimulator,
    backend_model,
)


def scenario(backend: str, model: str) -> ScenarioSpec:
    return ScenarioSpec(
        scenario_id="tracking-smoke",
        backend=BackendSpec(backend, model),
        simulation=SimulationSpec(dt=0.1, wheelbase=0.5),
        map_path="map.txt", path="path.csv",
        start=(1.0, 2.0), goal=(8.0, 9.0),
        planner="astar", controller="stanley",
        max_steps=100, seed=7,
    )


def artifact(spec: ScenarioSpec) -> RunArtifact:
    return RunArtifact(
        scenario=spec,
        metrics=RunMetrics(
            status="goal_reached", run_success=True, goal_reached=True,
            steps=20, goal_time_s=2.0, goal_distance=0.2,
            actual_path_length=7.5, control_effort=1.25,
        ),
        trace_csv=f"{spec.backend.name}.csv",
    )


class ExperimentSchemaTests(unittest.TestCase):
    def test_all_backend_scenarios_round_trip(self) -> None:
        for backend, model in (
                ("kinematic", "constrained_bicycle"),
                ("kinematic", "constrained_differential_drive"),
                ("mujoco", "planar"),
                ("pybullet", "racecar")):
            with self.subTest(backend=backend):
                original = scenario(backend, model)
                restored = ScenarioSpec.from_dict(original.to_dict())
                self.assertEqual(restored, original)

    def test_run_and_manifest_round_trip(self) -> None:
        runs = tuple(artifact(scenario(backend, model)) for backend, model in (
            ("kinematic", "constrained_bicycle"),
            ("mujoco", "planar"),
            ("pybullet", "planar"),
        ))
        manifest = ExperimentManifest("tracking-smoke", runs)
        restored = ExperimentManifest.from_dict(manifest.to_dict())
        self.assertEqual(restored, manifest)
        keys = [set(run.to_dict()) for run in restored.runs]
        self.assertEqual(keys[0], keys[1])
        self.assertEqual(keys[1], keys[2])

    def test_json_helpers_reject_unknown_version(self) -> None:
        spec = scenario("kinematic", "constrained_bicycle")
        data = spec.to_dict()
        data["schema_version"] = 99
        with self.assertRaisesRegex(ValueError, "unsupported"):
            ScenarioSpec.from_dict(data)

        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "run.json"
            original = artifact(spec)
            original.save_json(path)
            self.assertEqual(RunArtifact.load_json(path), original)
            self.assertEqual(original.to_dict()["goal_reached"], True)
            self.assertEqual(original.to_dict()["backend"], "kinematic")

    def test_invalid_backend_and_numeric_limits_are_rejected(self) -> None:
        with self.assertRaises(ValueError):
            BackendSpec("gazebo", "car")
        with self.assertRaises(ValueError):
            BackendSpec("mujoco", "racecar")
        with self.assertRaises(ValueError):
            SimulationSpec(dt=0.0)
        with self.assertRaises(ValueError):
            SimulationSpec(max_steering=-0.1)
        with self.assertRaises(ValueError):
            SimulationSpec(track_width=0.0)
        with self.assertRaises(ValueError):
            BackendSpec("kinematic", "unsupported_model")
        with self.assertRaises(ValueError):
            ScenarioSpec(
                scenario_id="bad", backend=BackendSpec("mujoco", "planar"),
                simulation=SimulationSpec(), map_path="", path="",
                start=(0.0, 0.0), goal=(1.0, 1.0), planner="astar",
                controller="stanley", max_steps=0)

    def test_differential_drive_limits_round_trip(self) -> None:
        original = SimulationSpec(
            dt=0.1, track_width=0.55, max_angular_velocity=2.5,
            max_angular_acceleration=3.0, max_wheel_velocity=2.0)
        restored = SimulationSpec.from_dict(original.to_dict())
        self.assertEqual(restored, original)

    def test_legacy_physics_summary_conversion(self) -> None:
        converted = artifact_from_legacy_summary({
            "backend": "mujoco", "physics_model": "planar_mujoco",
            "controller": "stanley", "wheelbase": 1.0,
            "steps": 10, "goal_reached": True, "goal_distance": 0.3,
            "actual_path_length": 4.5, "max_cross_track": 0.2,
            "mean_cross_track": 0.1, "max_heading_error": 0.08,
            "mean_heading_error": 0.04, "collision_steps": 0,
            "run_success": True, "csv": "trace.csv", "planner": {},
        })
        self.assertEqual(converted.scenario.backend, BackendSpec("mujoco", "planar"))
        self.assertEqual(converted.metrics.goal_time_s, 0.5)
        self.assertEqual(converted.trace_csv, "trace.csv")

    def test_backend_model_names_are_canonical(self) -> None:
        self.assertEqual(backend_model("kinematic", "racecar"),
                         "constrained_bicycle")
        self.assertEqual(backend_model("mujoco", "racecar"), "planar")
        self.assertEqual(backend_model("pybullet", "racecar"), "racecar")

    def test_kinematic_adapter_uses_common_protocol(self) -> None:
        class State:
            def __init__(self, x: float, y: float, theta: float, v: float):
                self.x, self.y, self.theta, self.v = x, y, theta, v

        class NativeSimulator:
            def __init__(self, initial: State, options: object):
                self.state = initial

            def reset(self, state: State) -> None:
                self.state = state

            def step(self, command: object) -> State:
                self.state = State(
                    self.state.x + command.velocity,
                    self.state.y, self.state.theta + command.steering,
                    command.velocity)
                return self.state

        fake = types.SimpleNamespace(
            State=State,
            Control=lambda velocity, steering: types.SimpleNamespace(
                velocity=velocity, steering=steering),
            SimulationOptions=type("SimulationOptions", (), {}),
            KinematicBicycleSimulator=NativeSimulator,
        )
        simulator = KinematicBicycleSimulator(PhysicsOptions(), fake)
        simulator.reset(1.0, 2.0, 0.0, 0.0)
        next_state = simulator.step(0.5, 0.1)
        self.assertEqual(next_state["x"], 1.5)
        self.assertEqual(next_state["theta"], 0.1)
        self.assertEqual(next_state["obstacle_contacts"], 0.0)


if __name__ == "__main__":
    unittest.main()
