"""Tests for the Python trajectory and control facade."""

from __future__ import annotations

import math
import unittest
from pathlib import Path

import robotnav


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "autoplanner" / "data" / "maps" / "simple_50x50.txt"


class ControlFacadeTests(unittest.TestCase):
    def test_plan_to_trajectory(self) -> None:
        planned = robotnav.plan(MAP, (1, 1), (12, 12))
        trajectory = robotnav.generate_trajectory(
            planned, robotnav.TrajectoryConfig(sample_spacing=0.75))
        self.assertGreater(len(trajectory.points), 2)
        self.assertGreater(trajectory.length, 0.0)
        self.assertEqual(trajectory.points[0].x, 1.0)

    def test_controller_and_simulator_return_structured_results(self) -> None:
        planned = robotnav.plan(MAP, (1, 1), (12, 12))
        trajectory = robotnav.generate_trajectory(planned)
        controller = robotnav.Controller(
            robotnav.ControllerConfig(controller="stanley"))
        result = robotnav.simulate(
            robotnav.RobotState(1.0, 1.0), trajectory, controller,
            max_time=1.0)
        self.assertTrue(result.states)
        self.assertEqual(len(result.states), len(result.controls))
        self.assertGreaterEqual(result.metrics.max_cross_track, 0.0)
        self.assertEqual(result.execution_model, "kinematic_bicycle")

    def test_differential_drive_execution_model(self) -> None:
        planned = robotnav.plan(MAP, (1, 1), (12, 12))
        trajectory = robotnav.generate_trajectory(planned)
        controller = robotnav.Controller(
            robotnav.ControllerConfig(controller="stanley"))
        config = robotnav.SimulationConfig(
            execution_model="differential_drive",
            track_width=0.55,
            max_angular_velocity=2.5,
            max_angular_acceleration=3.0,
            max_wheel_velocity=2.0,
        )

        result = robotnav.simulate(
            robotnav.RobotState(1.0, 1.0), trajectory, controller,
            config=config, max_time=1.0)

        self.assertTrue(result.states)
        self.assertEqual(len(result.states), len(result.controls))
        self.assertEqual(result.execution_model, "differential_drive")
        self.assertGreater(result.states[-1].x, result.states[0].x)

    def test_steering_to_twist_preserves_signed_curvature(self) -> None:
        steering = math.atan(0.5)
        forward = robotnav.steering_to_twist(
            robotnav.ControlCommand(2.0, steering), wheelbase=2.0)
        reverse = robotnav.steering_to_twist(
            robotnav.ControlCommand(-2.0, steering), wheelbase=2.0)

        self.assertAlmostEqual(forward.linear_velocity, 2.0)
        self.assertAlmostEqual(forward.angular_velocity, 0.5)
        self.assertAlmostEqual(reverse.angular_velocity, -0.5)

    def test_simulation_config_rejects_unknown_or_invalid_models(self) -> None:
        with self.assertRaises(ValueError):
            robotnav.SimulationConfig(execution_model="omnidirectional")
        with self.assertRaises(ValueError):
            robotnav.SimulationConfig(track_width=0.0)
        with self.assertRaises(ValueError):
            robotnav.steering_to_twist(
                robotnav.ControlCommand(1.0, 0.1), wheelbase=0.0)

    def test_motion_directions_are_forwarded(self) -> None:
        trajectory = robotnav.generate_trajectory(
            [(1, 1), (2, 1), (3, 1)],
            robotnav.TrajectoryConfig(allow_reverse=True),
            motion_directions=[1, 1, 1],
        )
        self.assertGreater(len(trajectory.points), 1)


if __name__ == "__main__":
    unittest.main()
