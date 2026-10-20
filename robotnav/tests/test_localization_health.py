"""Arrival-time localization-health and correction-gate regression tests."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

import robotnav

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "autoplanner" / "scripts"))
from simulate_sensor_navigation import build_parser, run
from benchmark_localization_health import run as run_benchmark


def estimate(stamp: int, position_std: float = 0.1,
             heading_std: float = 0.1) -> robotnav.StateEstimate:
    p = tuple(tuple((position_std if i < 2 else heading_std if i == 2 else 0.1) ** 2
                    if i == j else 0.0 for j in range(5)) for i in range(5))
    return robotnav.StateEstimate(stamp, robotnav.RobotState(2, 2), 0.0, p)


def evaluate(monitor: robotnav.LocalizationHealthMonitor, stamp: int,
             **kwargs) -> robotnav.LocalizationHealth:
    return monitor.evaluate(stamp, estimate(stamp), motion_timestamp_ns=stamp,
                            scan_timestamp_ns=stamp, **kwargs)


class LocalizationHealthTests(unittest.TestCase):
    def test_fault_benchmark_detects_failures_and_preserves_alias_limitation(self) -> None:
        first = run_benchmark()
        self.assertEqual(first, run_benchmark())
        cases = {case["case"]: case for case in first["cases"]}
        healthy = cases["healthy"]
        self.assertIsNone(healthy["first_anomaly_stop_step"])
        self.assertTrue(all(row["health"]["status"] == "tracking"
                            for row in healthy["trace"][1:]))
        for name in ("dynamic_only", "single_wall", "teleport", "lidar_outage", "motion_outage"):
            self.assertIsNotNone(cases[name]["first_anomaly_stop_step"])
            self.assertEqual(cases[name]["first_recovery_step"], 7)
            self.assertEqual(cases[name]["final_status"], "tracking")
        alias = cases["repeated_room_alias"]
        self.assertIsNone(alias["first_anomaly_stop_step"])
        self.assertEqual(alias["wrong_tracking_steps"], list(range(2, 10)))
        self.assertGreater(alias["max_position_error"], 9)

    def test_old_delayed_successes_do_not_override_stale_health(self) -> None:
        monitor = robotnav.LocalizationHealthMonitor()
        monitor.record_match(0, accepted=True, reason="matched")
        monitor.record_match(100_000_000, accepted=True, reason="matched")
        health = evaluate(monitor, 3_000_000_000)
        self.assertIn("map_correction_stale", health.reasons)
        self.assertTrue(health.safe_stop)
        monitor.record_match(200_000_000, accepted=True, reason="matched")
        monitor.record_match(300_000_000, accepted=True, reason="matched")
        self.assertTrue(evaluate(monitor, 3_000_000_000).safe_stop)

    def test_confirmation_failure_and_hysteresis(self) -> None:
        monitor = robotnav.LocalizationHealthMonitor()
        self.assertEqual(evaluate(monitor, 0).status, "initializing")
        monitor.record_match(0, accepted=True, reason="matched")
        self.assertTrue(evaluate(monitor, 0).safe_stop)
        monitor.record_match(100_000_000, accepted=True, reason="matched")
        self.assertEqual(evaluate(monitor, 100_000_000).status, "tracking")
        for index in range(2, 5):
            monitor.record_match(index * 100_000_000, accepted=False, reason="poor_fit")
            health = evaluate(monitor, index * 100_000_000)
            self.assertEqual(health.status, "degraded" if index < 4 else "lost")
            self.assertEqual(health.velocity_scale, 0.35 if index < 4 else 0.0)
            self.assertEqual(health.safe_stop, index == 4)
        self.assertTrue(health.safe_stop)
        monitor.record_match(500_000_000, accepted=True, reason="matched")
        self.assertEqual(evaluate(monitor, 500_000_000).status, "lost")
        monitor.record_match(600_000_000, accepted=True, reason="matched")
        self.assertEqual(evaluate(monitor, 600_000_000).status, "tracking")

    def test_stale_motion_scans_and_map_corrections(self) -> None:
        for motion, scan, expected in ((None, 0, "motion_stale"),
                                       (0, None, "scan_stale"),
                                       (0, 600_000_000, "motion_stale"),
                                       (600_000_000, 0, "scan_stale")):
            monitor = robotnav.LocalizationHealthMonitor(
                robotnav.LocalizationHealthConfig(require_map_matching=False))
            health = monitor.evaluate(600_000_000, estimate(600_000_000),
                                      motion_timestamp_ns=motion, scan_timestamp_ns=scan)
            self.assertIn(expected, health.reasons)
            self.assertTrue(health.safe_stop)
        monitor = robotnav.LocalizationHealthMonitor()
        self.assertIn("map_correction_stale", evaluate(monitor, 2_100_000_000).reasons)
        monitor.record_match(0, accepted=True, reason="matched")
        monitor.record_match(100_000_000, accepted=True, reason="matched")
        self.assertEqual(evaluate(monitor, 2_110_000_000).status, "lost")
        monitor.record_match(2_200_000_000, accepted=True, reason="matched")
        self.assertEqual(evaluate(monitor, 2_200_000_000).status, "lost")
        monitor.record_match(2_300_000_000, accepted=True, reason="matched")
        self.assertEqual(evaluate(monitor, 2_300_000_000).status, "tracking")

    def test_uncertainty_uses_xy_eigenvalue_and_heading(self) -> None:
        monitor = robotnav.LocalizationHealthMonitor(
            robotnav.LocalizationHealthConfig(require_map_matching=False))
        point = estimate(0, 0.8)
        p = [list(row) for row in point.covariance]
        p[0][1] = p[1][0] = 0.6
        health = monitor.evaluate(0, replace(point, covariance=tuple(map(tuple, p))),
                                  motion_timestamp_ns=0, scan_timestamp_ns=0)
        self.assertIn("position_uncertain", health.reasons)
        self.assertAlmostEqual(health.position_std, math.sqrt(1.24))
        health = monitor.evaluate(1, estimate(1, heading_std=0.6),
                                  motion_timestamp_ns=1, scan_timestamp_ns=1)
        self.assertIn("heading_uncertain", health.reasons)

    def test_correction_gate_wraps_heading(self) -> None:
        monitor = robotnav.LocalizationHealthMonitor()
        hint = robotnav.RobotState(2, 2, math.pi - 0.01)
        match = robotnav.ScanMatchResult(True, "matched",
            robotnav.RobotState(2, 2, -math.pi + 0.01), 0.1, 0.01, 20, 20, 0.5)
        self.assertTrue(monitor.assess_match(match, hint).accepted)
        for pose in (robotnav.RobotState(3, 2, hint.theta),
                     robotnav.RobotState(2, 2, hint.theta + 0.3)):
            assessment = monitor.assess_match(replace(match, pose=pose), hint)
            self.assertFalse(assessment.accepted)
            self.assertEqual(assessment.reason, "correction_jump")
        self.assertEqual(monitor.assess_match(replace(match, accepted=False, reason="poor_fit"),
                                              hint).reason, "poor_fit")

    def test_duplicate_older_and_long_gap_matches_do_not_restore_health(self) -> None:
        monitor = robotnav.LocalizationHealthMonitor()
        monitor.record_match(100, accepted=False, reason="poor_fit")
        self.assertFalse(monitor.record_match(100, accepted=True, reason="matched"))
        self.assertFalse(monitor.record_match(0, accepted=True, reason="matched"))
        self.assertEqual(monitor.ignored_matches, 2)
        monitor.record_match(200, accepted=True, reason="matched")
        monitor.record_match(3_000_000_000, accepted=True, reason="matched")
        self.assertEqual(evaluate(monitor, 3_000_000_000).status, "initializing")
        monitor.reset()
        self.assertEqual(monitor.ignored_matches, 0)
        self.assertEqual(evaluate(monitor, 0).consecutive_failures, 0)

    def test_dead_reckoning_does_not_require_map_matches(self) -> None:
        monitor = robotnav.LocalizationHealthMonitor(
            robotnav.LocalizationHealthConfig(require_map_matching=False))
        self.assertEqual(evaluate(monitor, 5_000_000_000).status, "tracking")

    def test_invalid_inputs_are_rejected_before_mutation(self) -> None:
        for kwargs in ({"failures_before_lost": True}, {"successes_before_recovery": 0},
                       {"max_motion_age": float("nan")}, {"degraded_velocity_scale": 2},
                       {"require_map_matching": 1}):
            with self.assertRaises(ValueError):
                robotnav.LocalizationHealthConfig(**kwargs)
        monitor = robotnav.LocalizationHealthMonitor()
        evaluate(monitor, 100)
        with self.assertRaises(ValueError):
            evaluate(monitor, 0)
        with self.assertRaises(ValueError):
            monitor.evaluate(200, estimate(100), motion_timestamp_ns=200, scan_timestamp_ns=200)
        with self.assertRaises(ValueError):
            monitor.record_match(200, accepted=1, reason="matched")
        for bad in (replace(estimate(100), covariance=((1.0,),)),
                    replace(estimate(100), covariance=tuple(
                        tuple(float("nan") if i == j == 0 else value
                              for j, value in enumerate(row))
                        for i, row in enumerate(estimate(100).covariance)))):
            with self.assertRaises(ValueError):
                monitor.evaluate(100, bad, motion_timestamp_ns=100, scan_timestamp_ns=100)
        self.assertEqual(evaluate(monitor, 100).status, "initializing")

    def test_navigation_stops_with_strict_uncertainty_limit_and_can_disable_health(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            args = build_parser().parse_args([
                "--steps", "5", "--beams", "11", "--localization", "dead_reckoning",
                "--latency-steps", "0", "--drop-probability", "0",
                "--max-position-std", "0.01", "--output", str(Path(temporary) / "trace.json")])
            guarded = run(args)
            args.localization_health = False
            unguarded = run(args)
        self.assertTrue(all(row["safe_stop"] for row in guarded["trace"]))
        self.assertTrue(all(row["command"]["velocity"] == 0 for row in guarded["trace"]))
        self.assertGreater(unguarded["trace"][-1]["robot"]["x"], 2.5)
        self.assertIn("position_uncertain", guarded["trace"][0]["localization_health"]["reasons"])

    def test_navigation_gates_jump_before_pose_fusion(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            args = build_parser().parse_args([
                "--steps", "7", "--beams", "31", "--latency-steps", "0",
                "--drop-probability", "0", "--initial-estimate-offset", "0.3", "0.2", "0.05",
                "--max-correction-translation", "0.000001",
                "--output", str(Path(temporary) / "trace.json")])
            guarded = run(args)
            args.localization_health = False
            unguarded = run(args)
        self.assertGreater(guarded["gated_matches"], 0)
        self.assertEqual(guarded["accepted_pose_corrections"], 0)
        self.assertEqual(guarded["safe_stop_steps"], 7)
        self.assertGreater(unguarded["accepted_pose_corrections"], 0)


if __name__ == "__main__":
    unittest.main()
