"""Known-map global search and ambiguity/ground-truth isolation contracts."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

import robotnav

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "autoplanner" / "scripts"))
from benchmark_relocalization import scenarios, run


def scan_at(rows, pose, noise=0.0):
    suite = robotnav.SensorSuite(rows, lidars=[robotnav.Lidar2DConfig(
        num_beams=61, field_of_view=2 * math.pi, max_range=25, ray_step=0.05,
        range_noise_std=noise)], seed=7)
    return suite.step(robotnav.WorldObservation(robotnav.SimulationClock(1, 0.5, 0.5),
        (robotnav.EntityObservation("robot", pose),))).ready[0]


class RelocalizationTests(unittest.TestCase):
    def test_noisy_unknown_hits_and_dynamic_exclusion(self):
        _, rows, truth, _ = scenarios()[0]
        scan = scan_at(rows, truth, noise=0.02)
        kinds = tuple("dynamic" if i % 10 == 0 else "unknown" for i in range(len(scan.ranges)))
        # Dynamic beams deliberately contain nonsense ranges; excluding them
        # must apply to both coarse ranking and final full-ray scoring.
        scan = replace(scan, kinds=kinds, ranges=tuple(100 if kind == "dynamic" else distance
                       for kind, distance in zip(kinds, scan.ranges)))
        result = robotnav.GridRelocalizer(rows).localize(scan)
        self.assertTrue(result.accepted)
        self.assertLess(math.hypot(result.pose.x - truth.x, result.pose.y - truth.y), 0.05)

    def test_persistent_teleport_recovery_ambiguity_and_explicit_reset_benchmark(self):
        result = run()
        cases = {case["case"]: case for case in result["cases"]}
        recovered = cases["persistent_teleport"]
        self.assertGreater(recovered["initial_position_error"], 5)
        self.assertFalse(recovered["local_match"]["accepted"])
        self.assertEqual(recovered["global_result"]["reason"], "localized")
        self.assertTrue(recovered["ekf_reset"])
        self.assertLess(recovered["final_position_error"], 0.05)
        for name in ("repeated_rooms", "symmetric_room"):
            case = cases[name]
            self.assertEqual(case["global_result"]["reason"], "ambiguous")
            self.assertIsNone(case["global_result"]["pose"])
            self.assertGreaterEqual(len(case["global_result"]["hypotheses"]), 2)
            self.assertFalse(case["ekf_reset"])
            self.assertEqual(case["initial_estimate"], case["final_estimate"])
        self.assertEqual(cases["dynamic_only"]["global_result"]["reason"], "insufficient_points")

    def test_poisoned_embedded_pose_does_not_change_search_and_order_is_repeatable(self):
        _, rows, truth, _ = scenarios()[0]
        scan = scan_at(rows, truth)
        matcher = robotnav.GridRelocalizer(rows)
        first = matcher.localize(scan)
        second = matcher.localize(replace(scan, pose=robotnav.RobotState(-999, 999, 2)))
        self.assertEqual(first, second)
        self.assertTrue(first.accepted)
        self.assertLess(abs(math.atan2(math.sin(first.pose.theta - truth.theta),
                                      math.cos(first.pose.theta - truth.theta))), 0.01)
        self.assertEqual(list(first.hypotheses), sorted(first.hypotheses,
                         key=lambda h: (h.ray_score, h.pose.x, h.pose.y, h.pose.theta)))
        self.assertGreater(first.coarse_poses, 0)
        self.assertLessEqual(first.refinements, matcher.config.max_refinements)

    def test_budget_refusal_before_partial_search(self):
        _, rows, truth, _ = scenarios()[0]
        scan = scan_at(rows, truth)
        for config in (robotnav.RelocalizationConfig(max_coarse_poses=2),
                       robotnav.RelocalizationConfig(translation_step=1e-308)):
            result = robotnav.GridRelocalizer(rows, config).localize(scan)
            self.assertEqual(result.reason, "search_budget_exceeded")
            self.assertIsNone(result.pose)
            self.assertEqual(result.hypotheses, ())
            self.assertEqual(result.refinements, 0)

    def test_no_hit_and_dynamic_scans_have_no_pose(self):
        _, rows, truth, _ = scenarios()[0]
        scan = scan_at(rows, truth)
        matcher = robotnav.GridRelocalizer(rows)
        for unsupported in (replace(scan, hits=(False,) * len(scan.ranges)),
                            replace(scan, kinds=("dynamic",) * len(scan.ranges))):
            result = matcher.localize(unsupported)
            self.assertFalse(result.accepted)
            self.assertEqual(result.reason, "insufficient_points")

    def test_no_hit_rays_penalize_unexpected_obstructions(self):
        matcher = robotnav.GridRelocalizer(["11111", "10001", "10001", "10001", "11111"])
        pose = robotnav.RobotState(2.5, 2.5)
        self.assertAlmostEqual(matcher._range(pose, 0, 5), 1.5)
        self.assertAlmostEqual(matcher._score([(0, 1.5, True)], pose), 0)
        self.assertGreater(matcher._score([(0, 3, False)], pose), 2)
        self.assertEqual(matcher._score([(0, 1, False)], pose), 0)

    def test_malformed_scan_and_config(self):
        _, rows, truth, _ = scenarios()[0]
        scan = scan_at(rows, truth)
        matcher = robotnav.GridRelocalizer(rows)
        for bad in (replace(scan, ranges=(float("nan"),) * len(scan.ranges)),
                    replace(scan, hits=(True,)), replace(scan, kinds=("static",))):
            with self.assertRaises(ValueError):
                matcher.localize(bad)
        for kwargs in ({"heading_bins": True}, {"max_hypotheses": 1},
                       {"translation_step": 0}, {"max_ray_score": float("nan")},
                       {"ambiguity_score_margin": -1}, {"max_refinements": 0}):
            with self.assertRaises(ValueError):
                robotnav.RelocalizationConfig(**kwargs)

    def test_strict_ray_consistency_rejects_instead_of_returning_pose(self):
        _, rows, truth, _ = scenarios()[0]
        result = robotnav.GridRelocalizer(rows, robotnav.RelocalizationConfig(
            max_ray_score=1e-12)).localize(scan_at(rows, truth))
        self.assertEqual(result.reason, "no_match")
        self.assertIsNone(result.pose)


if __name__ == "__main__":
    unittest.main()
