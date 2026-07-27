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

    def test_scene_round_trip_preserves_grid_settings_and_prediction(self) -> None:
        prediction = MovingObstaclePrediction(3, 5, 7, 8, 1, 0)
        grid = [[1, 1, 1], [1, 0, 1], [1, 1, 1]]
        settings = {"engine": "dynamic", "start_x": 1, "start_y": 1}
        with tempfile.TemporaryDirectory() as temp_dir:
            scene = Path(temp_dir) / "example.yaml"
            save_scene(scene, "autoplanner/data/maps/simple_50x50.txt",
                       grid, settings, prediction)
            loaded = load_scene(scene)
        self.assertEqual(loaded["grid"], grid)
        self.assertEqual(loaded["settings"], settings)
        self.assertEqual(loaded["prediction"], prediction)


if __name__ == "__main__":
    unittest.main()
