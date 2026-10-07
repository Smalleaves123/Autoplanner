from __future__ import annotations

import unittest
from pathlib import Path

import robotnav


ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / "autoplanner" / "data" / "maps" / "simple_50x50.txt"


class MultiAgentTests(unittest.TestCase):
    def test_cooperative_plan_avoids_vertex_conflicts(self) -> None:
        result = robotnav.plan_cooperative(
            MAP, [(2, 2), (8, 2)], [(8, 2), (2, 2)],
            allow_diagonal=False)
        self.assertTrue(result.success)
        self.assertEqual(result.planned_agents, 2)
        self.assertEqual(len(result.plans), 2)
        first, second = result.plans
        horizon = max(len(first.path), len(second.path))
        for index in range(horizon):
            self.assertNotEqual(
                first.path[min(index, len(first.path) - 1)],
                second.path[min(index, len(second.path) - 1)])

    def test_rejects_mismatched_agents(self) -> None:
        with self.assertRaises(ValueError):
            robotnav.plan_cooperative(MAP, [(1, 1)], [(2, 2), (3, 3)])

    def test_cbs_resolves_shared_destination(self) -> None:
        result = robotnav.plan_cooperative(
            MAP,
            [(2, 2), (8, 2), (2, 8)],
            [(5, 5), (5, 5), (5, 5)],
            allow_diagonal=False,
            algorithm="cbs",
        )
        self.assertTrue(result.success, result.message)
        self.assertEqual(result.planned_agents, 3)
        self.assertTrue(all(plan.success for plan in result.plans))

    def test_rejects_invalid_priority_order(self) -> None:
        with self.assertRaises(ValueError):
            robotnav.plan_cooperative(
                MAP, [(2, 2), (8, 2)], [(8, 2), (2, 2)],
                priority_order=[0, 0])


if __name__ == "__main__":
    unittest.main()
