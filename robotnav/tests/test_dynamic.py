"""Tests for the Python dynamic navigation facade."""

from __future__ import annotations

import unittest
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


if __name__ == "__main__":
    unittest.main()
