#!/usr/bin/env python3
"""Tests for deterministic robustness suite generation and perturbations."""

from __future__ import annotations

import argparse
import tempfile
import unittest
from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from validation_suites import (  # noqa: E402
    DynamicAgentSpec,
    PerturbationSpec,
    PerturbedSimulator,
    ValidationSuite,
    build_validation_suite,
    randomized_dynamic_agents,
)
from run_validation_suite import tracking_command  # noqa: E402


class FakeSimulator:
    def __init__(self) -> None:
        self.state = {"x": 0.0, "y": 0.0, "theta": 0.0, "v": 0.0,
                      "obstacle_contacts": 0.0}
        self.closed = False

    def reset(self, x: float, y: float, theta: float, velocity: float) -> None:
        self.state.update(x=x, y=y, theta=theta, v=velocity)

    def observe(self) -> dict[str, float]:
        return dict(self.state)

    def step(self, velocity: float, steering: float) -> dict[str, float]:
        self.state["x"] += velocity
        self.state["theta"] += steering
        self.state["v"] = velocity
        return self.observe()

    def close(self) -> None:
        self.closed = True


class ValidationSuiteTests(unittest.TestCase):
    def test_perturbation_rejects_invalid_values(self) -> None:
        with self.assertRaises(ValueError):
            PerturbationSpec(position_noise_std=-0.1)
        with self.assertRaises(ValueError):
            PerturbationSpec(observation_latency_steps=-1)
        with self.assertRaises(ValueError):
            PerturbationSpec(command_velocity_limit=0.0)

    def test_command_saturation_reports_applied_values(self) -> None:
        underlying = FakeSimulator()
        simulator = PerturbedSimulator(
            underlying,
            PerturbationSpec(
                command_velocity_limit=0.5, command_steering_limit=0.2),
            seed=4)
        simulator.reset(0.0, 0.0, 0.0, 0.0)
        result = simulator.step(2.0, -0.7)
        self.assertEqual(simulator.last_applied_velocity, 0.5)
        self.assertEqual(simulator.last_applied_steering, -0.2)
        self.assertEqual(result["x"], 0.5)
        self.assertEqual(result["theta"], -0.2)

    def test_observation_latency_delays_truth(self) -> None:
        simulator = PerturbedSimulator(
            FakeSimulator(), PerturbationSpec(observation_latency_steps=2),
            seed=2)
        simulator.reset(0.0, 0.0, 0.0, 0.0)
        self.assertEqual(simulator.step(1.0, 0.0)["x"], 0.0)
        self.assertEqual(simulator.step(1.0, 0.0)["x"], 0.0)
        self.assertEqual(simulator.step(1.0, 0.0)["x"], 1.0)

    def test_noise_is_reproducible_after_reset(self) -> None:
        simulator = PerturbedSimulator(
            FakeSimulator(), PerturbationSpec(position_noise_std=0.1), seed=9)
        simulator.reset(1.0, 2.0, 0.0, 0.0)
        first = simulator.observe()
        simulator.reset(1.0, 2.0, 0.0, 0.0)
        self.assertEqual(simulator.observe(), first)
        simulator.close()
        self.assertTrue(simulator.simulator.closed)

    def test_random_agents_are_seeded_free_and_non_overlapping(self) -> None:
        grid = tuple("0" * 12 for _ in range(10))
        first = randomized_dynamic_agents(grid, frames=10, count=3, seed=11)
        second = randomized_dynamic_agents(grid, frames=10, count=3, seed=11)
        self.assertEqual(first, second)
        tracks = []
        for agent in first:
            track = {
                (agent.x + agent.dx * step, agent.y + agent.dy * step)
                for step in range(agent.end_frame + 1)
            }
            self.assertTrue(all(
                0 <= x < 12 and 0 <= y < 10 for x, y in track))
            self.assertTrue(all(not (track & other) for other in tracks))
            tracks.append(track)

    def test_random_agents_fail_when_map_has_no_track(self) -> None:
        with self.assertRaises(ValueError):
            randomized_dynamic_agents(("111", "101", "111"), 5, 1, 0)

    def test_robustness_suite_has_all_case_kinds_and_unique_ids(self) -> None:
        grid = tuple("0" * 12 for _ in range(10))
        suite = build_validation_suite(
            "robustness", seed=5, repeats=2, grid=grid,
            frames=10, agent_count=2)
        self.assertEqual(len(suite.cases), 20)
        self.assertEqual(len({case.case_id for case in suite.cases}), 20)
        self.assertIn("dynamic_agents", {case.kind for case in suite.cases})
        self.assertTrue(any(case.perturbation.position_noise_std > 0.0
                            for case in suite.cases))
        self.assertTrue(any(case.perturbation.observation_latency_steps > 0
                            for case in suite.cases))
        self.assertTrue(any(case.perturbation.command_velocity_limit is not None
                            for case in suite.cases))

    def test_suite_json_round_trip(self) -> None:
        suite = build_validation_suite("latency", seed=3, repeats=1)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "suite.json"
            suite.save_json(path)
            restored = ValidationSuite.load_json(path)
        self.assertEqual(restored, suite)

    def test_dynamic_agent_validation_and_cli(self) -> None:
        with self.assertRaises(ValueError):
            DynamicAgentSpec(2, 1, 0, 0, 1, 0)
        agent = DynamicAgentSpec(0, 4, 1, 2, 1, 0)
        self.assertEqual(agent.cli_args(), [
            "--moving-obstacle", "0", "4", "1", "2", "1", "0"])

    def test_tracking_command_carries_scenario_and_perturbation(self) -> None:
        args = argparse.Namespace(
            backend="kinematic", controller="stanley", planner="astar",
            start=(2, 3), goal=(8, 9), velocity=0.7, steps=50, path=None)
        case = build_validation_suite("latency", seed=1).cases[1]
        command = tracking_command(
            Path("/repo"), Path("/repo/build"), Path("/repo/map.txt"),
            Path("/repo/out"), case, args)
        self.assertIn("--observation-latency-steps", command)
        self.assertIn("--start", command)
        self.assertIn("--goal", command)
        self.assertIn(case.case_id, command)


if __name__ == "__main__":
    unittest.main()
