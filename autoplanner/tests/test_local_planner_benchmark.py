#!/usr/bin/env python3
"""Regression tests for the local-planner benchmark helpers."""

import argparse
import sys
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import benchmark_local_planners as benchmark  # noqa: E402


class LocalPlannerBenchmarkTests(unittest.TestCase):
    def test_planner_args_are_selected_by_local_planner(self) -> None:
        args = argparse.Namespace(
            mppi_prediction_time=0.8,
            mppi_horizon=10,
            mppi_rollouts=32,
            mppi_temperature=0.5,
            mppi_velocity_noise=0.35,
            mppi_steering_noise=0.12,
            dwa_prediction_time=0.8,
            dwa_velocity_samples=7,
            dwa_steering_samples=9,
            dwa_dynamic_collision_samples=3,
        )

        self.assertIn("--mppi-rollouts", benchmark.planner_args("mppi", args))
        self.assertIn("32", benchmark.planner_args("mppi", args))
        self.assertIn("--dwa-velocity-samples", benchmark.planner_args("dwa", args))
        self.assertEqual(benchmark.planner_args("none", args), [])

    def test_report_contains_controller_and_local_planner(self) -> None:
        rows = [{
            "controller": "stanley",
            "local_planner": "mppi",
            "success": True,
            "goal_reached": True,
            "controller_trace_steps": 100,
            "local_planner_rollouts": 3200,
            "minimum_dynamic_obstacle_clearance": 0.8,
        }]

        report = benchmark.report(rows)

        self.assertIn("stanley", report)
        self.assertIn("mppi", report)
        self.assertIn("100.0%", report)


if __name__ == "__main__":
    unittest.main()
