"""Tests for the user-facing RobotNav planning facade."""

from __future__ import annotations

import unittest
from unittest import mock
from pathlib import Path

import robotnav


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "autoplanner" / "data" / "maps" / "simple_50x50.txt"


class PlanningFacadeTests(unittest.TestCase):
    def test_available_planners_comes_from_native_registry(self) -> None:
        import autoplanner

        self.assertEqual(
            robotnav.available_planners(),
            tuple(autoplanner.available_planners()),
        )
        self.assertIn("astar", robotnav.available_planners())

    def test_available_planners_has_pre_registry_fallback(self) -> None:
        import robotnav.planning as planning

        with mock.patch.object(
            planning,
            "_backend",
            side_effect=robotnav.BackendUnavailableError("unavailable"),
        ):
            self.assertIn("astar", planning.available_planners())

    def test_plan_from_path_returns_python_native_result(self) -> None:
        result = robotnav.plan(
            MAP, (1, 1), (20, 20),
            robotnav.PlannerConfig(planner="astar"),
        )
        self.assertTrue(result.success)
        self.assertEqual(result.status_code, "success")
        self.assertEqual(result.path[0], (1.0, 1.0))
        self.assertEqual(result.path[-1], (20.0, 20.0))
        self.assertGreater(result.path_length, 0.0)
        self.assertIsNotNone(result.native)

    def test_reusable_planner_and_native_map_are_supported(self) -> None:
        import autoplanner

        native_map = autoplanner.GridMap()
        self.assertTrue(native_map.load_from_txt(str(MAP)))
        planner = robotnav.Planner(robotnav.PlannerConfig(planner="jps"))
        result = planner.plan(native_map, (1, 1), (10, 10))
        self.assertTrue(result.success)
        self.assertEqual(result.planner_name, "jps")

    def test_invalid_inputs_fail_early(self) -> None:
        with self.assertRaises(FileNotFoundError):
            robotnav.plan("does-not-exist.txt", (1, 1), (2, 2))
        with self.assertRaises(ValueError):
            robotnav.PlannerConfig(max_iterations=0)
        with self.assertRaises(ValueError):
            robotnav.plan(MAP, (1,), (2, 2))

    def test_unknown_planner_is_reported_by_native_factory(self) -> None:
        with self.assertRaises(ValueError):
            robotnav.plan(
                MAP, (1, 1), (2, 2),
                robotnav.PlannerConfig(planner="not-a-planner"),
            )


if __name__ == "__main__":
    unittest.main()
