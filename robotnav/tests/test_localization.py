"""Known-map scan matching and event-time pose-correction regressions."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

import robotnav

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "autoplanner" / "scripts"))
from simulate_sensor_navigation import build_parser, run  # noqa: E402

ROOM = ["1" * 20] + ["1" + "0" * 18 + "1" for _ in range(14)] + ["1" * 20]


def observation(index: int, state: robotnav.RobotState) -> robotnav.WorldObservation:
    return robotnav.WorldObservation(
        robotnav.SimulationClock(index, index * 0.1, 0.1),
        (robotnav.EntityObservation("robot", state),))


def scan_at(state: robotnav.RobotState) -> robotnav.LidarScan:
    suite = robotnav.SensorSuite(ROOM, lidars=[robotnav.Lidar2DConfig(
        num_beams=91, field_of_view=2 * math.pi, max_range=25, ray_step=0.05)])
    return suite.step(observation(0, state)).ready[0]


class LocalizationTests(unittest.TestCase):
    def test_local_match_recovers_offset_without_embedded_truth(self) -> None:
        truth = robotnav.RobotState(5.4, 6.3, 0.12)
        scan = scan_at(truth)
        hint = robotnav.RobotState(5.8, 6.0, 0.2)
        matcher = robotnav.GridScanMatcher(ROOM)
        result = matcher.match(scan, hint)
        poisoned = replace(scan, pose=robotnav.RobotState(-999, 999, 2.0))
        self.assertEqual(result, matcher.match(poisoned, hint))
        self.assertTrue(result.accepted)
        self.assertLess(result.rmse, result.initial_rmse)
        self.assertAlmostEqual(result.pose.x, truth.x, delta=0.04)
        self.assertAlmostEqual(result.pose.y, truth.y, delta=0.04)
        self.assertAlmostEqual(result.pose.theta, truth.theta, delta=0.01)

    def test_dynamic_returns_are_excluded_and_unknown_hits_supported(self) -> None:
        truth = robotnav.RobotState(5.4, 6.3)
        scan = scan_at(truth)
        matcher = robotnav.GridScanMatcher(ROOM)
        dynamic = replace(scan, kinds=("dynamic",) * len(scan.ranges))
        result = matcher.match(dynamic, truth)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "insufficient_points")
        unknown = replace(scan, kinds=("unknown",) * len(scan.ranges))
        self.assertTrue(matcher.match(unknown, truth).accepted)

    def test_single_wall_is_rejected_as_unobservable(self) -> None:
        rows = ["1" * 11] + ["1" + "0" * 9 + "1" for _ in range(13)] + ["1" * 11]
        pose = robotnav.RobotState(5, 7)
        ys = list(range(2, 13))
        scan = robotnav.LidarScan(
            "lidar", "robot", 0, 0, 0, "map", pose,
            tuple(math.atan2(y - 7, 5) for y in ys),
            tuple(math.hypot(y - 7, 5) for y in ys), (True,) * len(ys))
        result = robotnav.GridScanMatcher(
            rows, robotnav.ScanMatcherConfig(min_points=6)).match(scan, pose)
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, "degenerate_geometry")

    def test_back_faces_and_occluded_surfaces_are_not_correspondences(self) -> None:
        rows = [row[:8] + "1" + row[9:] for row in ROOM]
        matcher = robotnav.GridScanMatcher(rows, robotnav.ScanMatcherConfig(
            max_correspondence_distance=2.0))
        self.assertTrue(matcher._visible(5.4, 6.3, 8.0, 6.5))
        self.assertFalse(matcher._visible(5.4, 6.3, 19.0, 6.5))
        nearest = matcher._nearest(9.02, 6.5, 5.4, 6.3)
        self.assertIsNotNone(nearest)
        self.assertEqual(nearest[0], 8.0)  # not the invisible back face x=9
        self.assertFalse(matcher._visible(8.5, 6.3, 8.0, 6.5))

    def test_bad_scan_and_configuration_are_rejected(self) -> None:
        scan = scan_at(robotnav.RobotState(5.4, 6.3))
        matcher = robotnav.GridScanMatcher(ROOM)
        for bad in (replace(scan, ranges=(float("nan"),) * len(scan.ranges)),
                    replace(scan, hits=(True,)), replace(scan, kinds=("static",))):
            with self.assertRaises(ValueError):
                matcher.match(bad, robotnav.RobotState(5.4, 6.3))
        for kwargs in ({"min_points": 1}, {"max_rmse": -1},
                       {"min_inlier_ratio": 2}, {"iterations": True}):
            with self.assertRaises(ValueError):
                robotnav.ScanMatcherConfig(**kwargs)
        unsupported = replace(scan, ranges=(1.0,) * len(scan.ranges))
        self.assertFalse(matcher.match(unsupported, robotnav.RobotState(5.4, 6.3)).accepted)

    def test_delayed_pose_correction_survives_motion_replay(self) -> None:
        packets = [robotnav.OdometryMeasurement(
            "odom", "robot", seq, seq * 100_000_000, seq * 100_000_000,
            0.0, 0.0, 0.0, 1.0, 0.1 if seq else 0.0) for seq in range(11)]
        correction = robotnav.PoseMeasurement(
            "map", "robot", 0, 400_000_000, 1_200_000_000, 0.5, 0.1, 0.02)
        first = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0, velocity=1.0))
        late = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0, velocity=1.0))
        for packet in packets[:5]:
            first.update(packet)
        first.update(correction)
        for packet in packets[5:]:
            first.update(packet)
        for packet in reversed(packets):
            late.update(packet)
        self.assertTrue(late.update(correction))
        self.assertEqual(first.estimate, late.estimate)
        self.assertEqual(late.accepted_pose_packets, 1)
        self.assertEqual(late.accepted_motion_packets, 11)
        self.assertFalse(late.update(correction))
        self.assertEqual(late.rejected_pose_packets, 1)

    def test_pose_heading_innovation_wraps_at_pi(self) -> None:
        initial = robotnav.RobotState(1.0, 1.0, math.pi - 0.01)
        ekf = robotnav.PlanarEkf(initial)
        ekf.update(robotnav.PoseMeasurement(
            "map", "robot", 0, 0, 0, 1.0, 1.0, -math.pi + 0.02))
        difference = ekf.estimate.state.theta - initial.theta
        self.assertLess(abs(math.atan2(math.sin(difference), math.cos(difference))), 0.04)
        before = ekf.estimate
        with self.assertRaises(ValueError):
            ekf.update(robotnav.PoseMeasurement(
                "map", "robot", 1, 1, 1, 1.0, 1.0, 0.0, position_std=0))
        self.assertEqual(ekf.estimate, before)

    def test_same_biased_sensor_sequence_reduces_dead_reckoning_drift(self) -> None:
        suite = robotnav.SensorSuite(
            ROOM, lidars=[robotnav.Lidar2DConfig(
                num_beams=61, field_of_view=2 * math.pi, max_range=25, ray_step=0.05)],
            odometries=[robotnav.OdometryConfig()], imus=[robotnav.ImuConfig()])
        matcher = robotnav.GridScanMatcher(ROOM)
        initial = robotnav.RobotState(5.7, 6.5, 0.06, 0.5)
        baseline, fused = robotnav.PlanarEkf(initial), robotnav.PlanarEkf(initial)
        for seq in range(40):
            truth = robotnav.RobotState(5.4 + 0.05 * seq, 6.3, 0.0, 0.5)
            packets = suite.step(observation(seq, truth)).ready
            for packet in packets:
                if isinstance(packet, robotnav.OdometryMeasurement):
                    packet = replace(packet, velocity=packet.velocity * 1.15,
                                     delta_theta=packet.delta_theta + 0.02 * packet.sample_dt)
                elif isinstance(packet, robotnav.ImuMeasurement):
                    packet = replace(packet, angular_velocity=packet.angular_velocity + 0.02)
                else:
                    continue
                baseline.update(packet)
                fused.update(packet)
            if seq % 5 == 0:
                scan = next(item for item in packets if isinstance(item, robotnav.LidarScan))
                match = matcher.match(scan, fused.estimate.state)
                if match.accepted:
                    fused.update(robotnav.PoseMeasurement(
                        "map", "robot", seq, scan.timestamp_ns, scan.arrival_timestamp_ns,
                        match.pose.x, match.pose.y, match.pose.theta))
        def error(state: robotnav.RobotState) -> float:
            return math.hypot(state.x - truth.x, state.y - truth.y)
        self.assertGreater(fused.accepted_pose_packets, 0)
        self.assertLess(error(fused.estimate.state), 0.5 * error(baseline.estimate.state))

    def test_navigation_match_corrections_are_repeatable_and_optional(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            args = build_parser().parse_args([
                "--steps", "16", "--beams", "31", "--goal", "8", "2",
                "--initial-estimate-offset", "0.3", "0.2", "0.05",
                "--output", str(Path(temporary) / "trace.json")])
            first, second = run(args), run(args)
            args.localization = "dead_reckoning"
            baseline = run(args)
        self.assertEqual(first, second)
        self.assertGreater(first["accepted_pose_corrections"], 0)
        self.assertEqual(baseline["scan_match_attempts"], 0)
        self.assertLess(first["position_rmse"], baseline["position_rmse"])


if __name__ == "__main__":
    unittest.main()
