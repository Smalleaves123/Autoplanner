#!/usr/bin/env python3
"""Unit tests for the dashboard scenario and prediction helpers."""

import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from dashboard_support import (  # noqa: E402
    MovingObstaclePrediction,
    decide_prediction_risk,
    load_scene,
    save_scene,
)
from navigation_dashboard import build_command  # noqa: E402
from run_saved_scenarios import run_command  # noqa: E402


class DashboardSupportTests(unittest.TestCase):
    def test_risk_decision_distinguishes_monitor_replan_and_stop(self) -> None:
        start = (1, 1)
        goal = (20, 1)
        monitor = decide_prediction_risk(
            MovingObstaclePrediction(1, 3, 10, 10), start, goal, 1.0)
        replan = decide_prediction_risk(
            MovingObstaclePrediction(1, 3, 10, 2), start, goal, 1.0)
        stop = decide_prediction_risk(
            MovingObstaclePrediction(1, 3, 1, 1), start, goal, 1.0)
        self.assertEqual(monitor.action, "monitor")
        self.assertEqual(replan.action, "replan_slow")
        self.assertLess(replan.velocity_scale, 1.0)
        self.assertEqual(stop.action, "safe_stop")
        self.assertEqual(stop.velocity_scale, 0.0)

    def test_prediction_rejects_reversed_frame_range(self) -> None:
        prediction = MovingObstaclePrediction(4, 3, 7, 8)
        with self.assertRaises(ValueError):
            prediction.positions()

    def test_prediction_rejects_negative_footprint_parameters(self) -> None:
        with self.assertRaises(ValueError):
            MovingObstaclePrediction(1, 2, 7, 8, radius=-0.1)
        with self.assertRaises(ValueError):
            MovingObstaclePrediction(
                1, 2, 7, 8, uncertainty_growth_per_frame=-0.01)

    def test_prediction_supports_acceleration_and_covariance(self) -> None:
        prediction = MovingObstaclePrediction(
            0, 4, 1, 2, 1, 0,
            acceleration_x_per_frame2=0.5,
            covariance_xx=1.0,
            covariance_yy=4.0,
            covariance_growth_xx_per_frame=0.25,
            covariance_confidence_scale=2.0,
        )
        self.assertEqual(prediction.positions()[2], (4.0, 2.0))
        self.assertAlmostEqual(prediction.safety_radius_at_frame(2), 4.0)
        with self.assertRaises(ValueError):
            MovingObstaclePrediction(0, 1, 0, 0, covariance_xx=1.0,
                                      covariance_xy=2.0,
                                      covariance_yy=1.0)

    def test_risk_decision_includes_obstacle_footprint_and_uncertainty(self) -> None:
        start = (1, 1)
        goal = (20, 1)
        point_prediction = MovingObstaclePrediction(1, 3, 10, 5)
        footprint_prediction = MovingObstaclePrediction(
            1, 3, 10, 5, radius=1.0, uncertainty_growth_per_frame=0.5)
        point_decision = decide_prediction_risk(
            point_prediction, start, goal, 1.0)
        footprint_decision = decide_prediction_risk(
            footprint_prediction, start, goal, 1.0)
        self.assertEqual(point_decision.action, "monitor")
        self.assertEqual(footprint_decision.action, "replan_slow")
        self.assertLess(footprint_decision.minimum_clearance,
                        point_decision.minimum_clearance)

    def test_scene_round_trip_preserves_grid_settings_and_prediction(self) -> None:
        prediction = MovingObstaclePrediction(
            3, 5, 7, 8, 1, 0, radius=0.75,
            uncertainty_growth_per_frame=0.1)
        grid = [[1, 1, 1], [1, 0, 1], [1, 1, 1]]
        settings = {
            "engine": "dynamic", "start_x": 1, "start_y": 1,
            "prediction_risk_weight": 2.5,
            "prediction_risk_clearance": 3.0,
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            scene = Path(temp_dir) / "example.yaml"
            save_scene(scene, "autoplanner/data/maps/simple_50x50.txt",
                       grid, settings, prediction)
            loaded = load_scene(scene)
        self.assertEqual(loaded["grid"], grid)
        self.assertEqual(loaded["settings"], settings)
        self.assertEqual(loaded["prediction"], prediction)

    def test_old_six_value_prediction_scene_remains_compatible(self) -> None:
        scene_text = """\
schema_version: 1
base_map: autoplanner/data/maps/simple_50x50.txt
grid:
  - '00'
  - '00'
settings:
  engine: dynamic
  use_prediction: true
prediction:
  start_frame: 1
  end_frame: 3
  x: 1
  y: 1
  dx: 0
  dy: 1
"""
        with tempfile.TemporaryDirectory() as temp_dir:
            scene = Path(temp_dir) / "old.yaml"
            scene.write_text(scene_text)
            loaded = load_scene(scene)
        self.assertEqual(
            loaded["prediction"], MovingObstaclePrediction(1, 3, 1, 1, 0, 1))

    def test_dashboard_command_forwards_prediction_safety_parameters(self) -> None:
        prediction = MovingObstaclePrediction(
            2, 5, 7, 8, 1, 0, radius=0.75,
            uncertainty_growth_per_frame=0.1,
            acceleration_x_per_frame2=0.2,
            covariance_xx=0.25,
            covariance_yy=0.25,
            covariance_growth_xx_per_frame=0.05,
            covariance_growth_yy_per_frame=0.05,
            covariance_confidence_scale=1.5)
        settings = {
            "engine": "dynamic",
            "map": Path("map.txt"),
            "planner": "improved_astar",
            "controller": "stanley",
            "start_x": 1,
            "start_y": 1,
            "goal_x": 10,
            "goal_y": 10,
            "effective_velocity": 0.75,
            "robot_radius": 1.0,
            "smooth": "none",
            "effective_frames": 20,
            "steps_per_frame": 40,
            "obstacle_ahead": 5,
            "auto_obstacles": False,
            "prediction": prediction,
            "prediction_risk_weight": 2.5,
            "prediction_risk_clearance": 3.0,
        }
        command = build_command(settings, Path("results/run"))
        self.assertIn("--moving-obstacle-radius", command)
        self.assertIn("0.75", command)
        self.assertIn("--moving-obstacle-uncertainty-growth", command)
        self.assertIn("0.1", command)
        self.assertIn("--prediction-risk-weight", command)
        self.assertIn("2.5", command)
        self.assertIn("--prediction-risk-clearance", command)
        self.assertIn("3.0", command)
        self.assertIn("--moving-obstacle-acceleration", command)
        self.assertIn("0.2", command)
        self.assertIn("--moving-obstacle-covariance", command)
        self.assertIn("0.25", command)
        self.assertIn("--moving-obstacle-covariance-growth", command)
        self.assertIn("0.05", command)
        self.assertIn("--moving-obstacle-confidence-scale", command)
        self.assertIn("1.5", command)

    def test_saved_scenario_command_forwards_prediction_safety_parameters(self) -> None:
        scene = {
            "grid": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            "settings": {
                "engine": "dynamic",
                "planner": "improved_astar",
                "controller": "stanley",
                "start_x": 0,
                "start_y": 0,
                "goal_x": 2,
                "goal_y": 0,
                "velocity": 1.0,
                "robot_radius": 0.0,
                "frames": 4,
                "steps_per_frame": 10,
                "obstacle_ahead": 2,
                "auto_obstacles": False,
                "prediction_risk_weight": 2.5,
                "prediction_risk_clearance": 3.0,
            },
            "prediction": MovingObstaclePrediction(
                1, 2, 1, 2, radius=0.5,
                uncertainty_growth_per_frame=0.1),
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            command, metadata = run_command(
                scene, Path(temp_dir) / "run", "build", False)
        self.assertEqual(metadata["decision"], "replan_slow")
        self.assertEqual(metadata["moving_obstacle_radius"], 0.5)
        self.assertEqual(metadata["prediction_risk_weight"], 2.5)
        self.assertIn("--moving-obstacle-radius", command)
        self.assertIn("--moving-obstacle-uncertainty-growth", command)
        self.assertIn("--prediction-risk-weight", command)
        self.assertIn("--prediction-risk-clearance", command)


if __name__ == "__main__":
    unittest.main()
