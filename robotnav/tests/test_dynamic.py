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
        self.assertIn("final_state", result.metrics)
        self.assertIn("local_planner_warm_start_count", result.metrics)
        self.assertIn("mean_mppi_effective_sample_ratio", result.metrics)
        self.assertIn("maximum_mppi_collision_probability", result.metrics)
        self.assertIn("navigation_state", result.trace[0])
        self.assertTrue(result.state_transitions)
        self.assertIn("reason", result.state_transitions[0])
        self.assertIsInstance(result.path, tuple)

    def test_dynamic_config_validates_ranges(self) -> None:
        with self.assertRaises(ValueError):
            robotnav.DynamicConfig(frames=0)
        with self.assertRaises(ValueError):
            robotnav.MovingObstacle(3, 2, 1, 1)
        with self.assertRaises(ValueError):
            robotnav.DynamicConfig(max_replanning_retries=-1)
        with self.assertRaises(ValueError):
            robotnav.DynamicConfig(replanning_cooldown_frames=-1)
        with self.assertRaises(ValueError):
            robotnav.DynamicConfig(recovery_stop_steps=0)

    def test_scenario_and_result_json_round_trip(self) -> None:
        scenario = robotnav.DynamicScenario(
            MAP, (1, 1), (4, 4),
            robotnav.DynamicConfig(
                frames=1,
                steps_per_frame=2,
                max_replanning_retries=3,
                replanning_cooldown_frames=2,
                recovery_stop_steps=4,
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
            self.assertEqual(restored.config.max_replanning_retries, 3)
            self.assertEqual(restored.config.replanning_cooldown_frames, 2)
            self.assertEqual(restored.config.recovery_stop_steps, 4)
            result = robotnav.run_dynamic_scenario(restored)
            result.save_json(result_path)
            loaded = robotnav.DynamicResult.load_json(result_path)
            self.assertEqual(loaded.status_code, result.status_code)
            self.assertEqual(loaded.path, result.path)
            self.assertEqual(loaded.state_transitions,
                             result.state_transitions)


if __name__ == "__main__":
    unittest.main()
