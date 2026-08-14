"""Tests for the Python dynamic navigation facade."""

from __future__ import annotations

import unittest
import tempfile
from pathlib import Path

import robotnav


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "autoplanner" / "data" / "maps" / "simple_50x50.txt"


class DynamicFacadeTests(unittest.TestCase):
    def test_dynamic_run_returns_trace_and_metrics(self) -> None:
        result = robotnav.run_dynamic(
            MAP, (1, 1), (10, 10),
            robotnav.DynamicConfig(
                planner="space_time_astar",
                controller="stanley",
                frames=3,
                steps_per_frame=8,
                auto_insert_obstacles=False,
                moving_obstacles=(robotnav.MovingObstacle(
                    0, 2, 7, 4, 0, 1, radius=0.25),),
            ),
        )
        self.assertIn(result.status_code, {"success", "timeout"})
        self.assertTrue(result.trace)
        self.assertIn("replanning_count", result.metrics)
        self.assertIsInstance(result.path, tuple)

    def test_dynamic_config_validates_ranges(self) -> None:
        with self.assertRaises(ValueError):
            robotnav.DynamicConfig(frames=0)
        with self.assertRaises(ValueError):
            robotnav.MovingObstacle(3, 2, 1, 1)

    def test_scenario_and_result_json_round_trip(self) -> None:
        scenario = robotnav.DynamicScenario(
            MAP, (1, 1), (4, 4),
            robotnav.DynamicConfig(
                frames=1,
                steps_per_frame=2,
                moving_obstacles=(robotnav.MovingObstacle(0, 1, 7, 4),),
            ),
        )
        with tempfile.TemporaryDirectory() as directory:
            scenario_path = Path(directory) / "scenario.json"
            result_path = Path(directory) / "result.json"
            scenario.save_json(scenario_path)
            restored = robotnav.DynamicScenario.load_json(scenario_path)
            self.assertEqual(restored.start, scenario.start)
            self.assertEqual(restored.config.moving_obstacles,
                             scenario.config.moving_obstacles)
            result = robotnav.run_dynamic_scenario(restored)
            result.save_json(result_path)
            loaded = robotnav.DynamicResult.load_json(result_path)
            self.assertEqual(loaded.status_code, result.status_code)
            self.assertEqual(loaded.path, result.path)


if __name__ == "__main__":
    unittest.main()
