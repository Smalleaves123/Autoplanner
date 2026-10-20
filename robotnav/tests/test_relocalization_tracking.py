"""Temporal confirmation, relative motion, and delayed-packet epoch contracts."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

import robotnav

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "autoplanner" / "scripts"))
from benchmark_relocalization_tracking import run


def result(pose, other=None, accepted=True):
    hypotheses = tuple(robotnav.RelocalizationHypothesis(
        p, 0.001 if i == 0 else 0.002,
        robotnav.ScanMatchResult(True, "matched", p, 0.1, 0.01, 20, 20, 0.5))
        for i, p in enumerate((pose,) if other is None else (pose, other)))
    return robotnav.RelocalizationResult(accepted, "localized" if accepted else "ambiguous",
        pose if accepted else None, hypotheses, 100, 10, 0.001 if other else None)


def confirmed(pose=robotnav.RobotState(8, 6)):
    tracker = robotnav.RelocalizationTracker()
    for stamp in (0, 100_000_000, 200_000_000):
        decision = tracker.update(result(pose), timestamp_ns=stamp, motion_pose=robotnav.RobotState(0, 0))
    return decision


def imu(stamp, sequence=0):
    return robotnav.ImuMeasurement("imu", "robot", sequence, stamp, stamp + 100_000_000, 0, 0.08)


class RelocalizationTrackingTests(unittest.TestCase):
    def test_actual_multiscan_global_search_and_epoch_handoff(self):
        cases = {case["case"]: case for case in run()["cases"]}
        recovered = cases["stationary_recovery"]
        self.assertEqual(recovered["first_confirmation_step"], 2)
        self.assertEqual(recovered["epoch_id"], 1)
        self.assertEqual(recovered["rejected_queued_packets"], 3)
        self.assertLess(recovered["final_position_error"], 0.05)
        self.assertEqual(cases["moving_confirmation"]["first_confirmation_step"], 2)
        self.assertEqual(cases["moving_confirmation"]["epoch_id"], 0)
        self.assertLess(cases["moving_confirmation"]["confirmation_position_error"], 0.05)
        self.assertIsNone(cases["persistent_ambiguity"]["first_confirmation_step"])
        self.assertTrue(all(row["decision"]["status"] == "ambiguous"
                            for row in cases["persistent_ambiguity"]["trace"]))

    def test_three_unique_motion_consistent_frames_confirm(self):
        tracker = robotnav.RelocalizationTracker()
        first = robotnav.RobotState(5, 6, math.pi / 2)
        second = robotnav.RobotState(5, 6.2, math.pi / 2 + 0.05)
        third = robotnav.RobotState(5 - 0.2 * math.sin(0.05),
                                   6.2 + 0.2 * math.cos(0.05), math.pi / 2 + 0.1)
        motion = (robotnav.RobotState(0, 0), robotnav.RobotState(0.2, 0, 0.05),
                  robotnav.RobotState(0.2 + 0.2 * math.cos(0.05), 0.2 * math.sin(0.05), 0.1))
        decisions = [tracker.update(result(p), timestamp_ns=i * 100_000_000, motion_pose=m)
                     for i, (p, m) in enumerate(zip((first, second, third), motion))]
        self.assertEqual([d.status for d in decisions], ["candidate_pending", "candidate_pending", "confirmed"])
        self.assertEqual(decisions[-1].pose, third)
        self.assertEqual(len({d.tracks[0].track_id for d in decisions}), 1)
        self.assertEqual(decisions[-1].tracks[0].unique_frames, 3)

    def test_ambiguous_modes_remain_ambiguous_and_need_new_unique_streak(self):
        tracker = robotnav.RelocalizationTracker()
        a, b = robotnav.RobotState(5, 6), robotnav.RobotState(15, 6)
        for index in range(4):
            decision = tracker.update(result(a, b, False), timestamp_ns=index * 100_000_000,
                                      motion_pose=robotnav.RobotState(0, 0))
            self.assertEqual(decision.status, "ambiguous")
            self.assertIsNone(decision.pose)
            self.assertTrue(all(t.unique_frames == 0 for t in decision.tracks))
        self.assertEqual([t.consistent_frames for t in decision.tracks], [4, 4])
        for index in range(4, 7):
            decision = tracker.update(result(a), timestamp_ns=index * 100_000_000,
                                      motion_pose=robotnav.RobotState(0, 0))
            self.assertEqual(decision.status, "confirmed" if index == 6 else "candidate_pending")

    def test_failures_long_gap_and_motion_inconsistency_break_confirmation(self):
        pose = robotnav.RobotState(5, 6)
        tracker = robotnav.RelocalizationTracker()
        for stamp in (0, 100_000_000):
            tracker.update(result(pose), timestamp_ns=stamp, motion_pose=robotnav.RobotState(0, 0))
        missing = robotnav.RelocalizationResult(False, "no_match", None, (), 100, 24, None)
        self.assertEqual(tracker.update(missing, timestamp_ns=200_000_000,
                         motion_pose=robotnav.RobotState(0, 0)).status, "no_match")
        decision = tracker.update(result(pose), timestamp_ns=300_000_000, motion_pose=robotnav.RobotState(0, 0))
        self.assertEqual(decision.tracks[0].unique_frames, 1)
        decision = tracker.update(result(pose), timestamp_ns=2_000_000_000, motion_pose=robotnav.RobotState(0, 0))
        self.assertEqual(decision.status, "gap_reset")
        jump = tracker.update(result(replace(pose, x=9)), timestamp_ns=2_100_000_000,
                              motion_pose=robotnav.RobotState(0, 0))
        self.assertEqual(jump.status, "motion_inconsistent")
        self.assertEqual(jump.tracks[0].unique_frames, 1)

    def test_duplicates_older_frames_heading_wrap_and_reset(self):
        tracker = robotnav.RelocalizationTracker()
        pose = robotnav.RobotState(5, 6, math.pi - 0.01)
        first = tracker.update(result(pose), timestamp_ns=100, motion_pose=robotnav.RobotState(0, 0))
        for stamp in (100, 0):
            duplicate = tracker.update(result(pose), timestamp_ns=stamp, motion_pose=robotnav.RobotState(0, 0))
            self.assertEqual(duplicate.status, "out_of_order")
            self.assertEqual(duplicate.tracks, first.tracks)
        next_pose = replace(pose, theta=-math.pi + 0.01)
        second = tracker.update(result(next_pose), timestamp_ns=200,
                                motion_pose=robotnav.RobotState(0, 0, 0.02))
        self.assertEqual(first.tracks[0].track_id, second.tracks[0].track_id)
        tracker.reset()
        self.assertEqual(tracker.ignored_frames, 0)
        self.assertEqual(tracker.update(result(pose), timestamp_ns=0,
                         motion_pose=robotnav.RobotState(0, 0)).tracks[0].unique_frames, 1)

    def test_epoch_reset_preserves_bias_means_and_rejects_old_delayed_data(self):
        epoch = robotnav.RelocalizationEpoch(robotnav.RobotState(2, 2), robotnav.EkfConfig(
            estimate_gyro_bias=True, estimate_acceleration_bias=True,
            initial_gyro_bias=0.01, initial_acceleration_bias=0.1))
        epoch.update_motion(imu(100_000_000), epoch_id=0)
        before = epoch.ekf.estimate
        transition = epoch.apply(confirmed(), now_ns=250_000_000, epoch_id=0, stationary=True)
        self.assertEqual(transition.epoch_id, 1)
        self.assertEqual(epoch.ekf.estimate.state, robotnav.RobotState(8, 6))
        self.assertEqual(epoch.ekf.estimate.timestamp_ns, 250_000_000)
        self.assertEqual(epoch.ekf.estimate.gyro_bias, before.gyro_bias)
        self.assertEqual(epoch.ekf.estimate.acceleration_bias, before.acceleration_bias)
        self.assertEqual(len(epoch.ekf.estimate.covariance), 7)
        self.assertEqual(epoch.ekf.accepted_packets, 0)
        self.assertEqual(epoch.health.latest_timestamp_ns, 250_000_000)
        self.assertIn("mapping", transition.required_resets)
        health = epoch.health.evaluate(250_000_000, epoch.ekf.estimate,
                                      motion_timestamp_ns=None, scan_timestamp_ns=None)
        self.assertTrue(health.safe_stop)
        after = epoch.ekf.estimate
        self.assertFalse(epoch.update_motion(imu(100_000_000), epoch_id=0))
        self.assertFalse(epoch.update_motion(imu(400_000_000, 1), epoch_id=0))
        self.assertFalse(epoch.update_motion(imu(250_000_000, 2), epoch_id=1))
        self.assertEqual(epoch.ekf.estimate, after)
        self.assertTrue(epoch.update_motion(imu(300_000_000, 3), epoch_id=1))
        self.assertEqual(epoch.rejected_epoch_packets, 3)
        epoch.health.record_match(300_000_000, accepted=True, reason="matched")
        self.assertTrue(epoch.health.evaluate(300_000_000, epoch.ekf.estimate_at(300_000_000),
                        motion_timestamp_ns=300_000_000, scan_timestamp_ns=300_000_000).safe_stop)
        epoch.health.record_match(320_000_000, accepted=True, reason="matched")
        self.assertEqual(epoch.health.evaluate(320_000_000, epoch.ekf.estimate_at(320_000_000),
                         motion_timestamp_ns=300_000_000, scan_timestamp_ns=320_000_000).status, "tracking")
        with self.assertRaises(ValueError):
            epoch.apply(replace(confirmed(), timestamp_ns=320_000_000),
                        now_ns=350_000_000, epoch_id=0, stationary=True)
        with self.assertRaises(ValueError):
            epoch.apply(confirmed(), now_ns=350_000_000, epoch_id=1, stationary=True)

    def test_epoch_refuses_pending_stale_future_nonstationary_and_old_now(self):
        for decision, now, stationary in (
            (replace(confirmed(), status="candidate_pending", pose=None), 250_000_000, True),
            (confirmed(), 1_000_000_000, True), (confirmed(), 100_000_000, True),
            (confirmed(), 250_000_000, False)):
            epoch = robotnav.RelocalizationEpoch(robotnav.RobotState(2, 2))
            before = epoch.ekf.estimate
            with self.assertRaises(ValueError):
                epoch.apply(decision, now_ns=now, epoch_id=0, stationary=stationary)
            self.assertEqual(epoch.epoch_id, 0)
            self.assertEqual(epoch.ekf.estimate, before)
        epoch.health.evaluate(300_000_000, epoch.ekf.estimate_at(300_000_000),
                              motion_timestamp_ns=None, scan_timestamp_ns=None)
        with self.assertRaises(ValueError):
            epoch.apply(confirmed(), now_ns=250_000_000, epoch_id=0, stationary=True)

    def test_lidar_epoch_filter_and_wrong_entity(self):
        epoch = robotnav.RelocalizationEpoch(robotnav.RobotState(2, 2))
        epoch.apply(confirmed(), now_ns=250_000_000, epoch_id=0, stationary=True)
        scan = robotnav.LidarScan("lidar", "robot", 0, 300_000_000, 500_000_000,
                                "map", robotnav.RobotState(0, 0), (0,), (2,), (True,))
        self.assertFalse(epoch.accept_packet(scan, epoch_id=0))
        self.assertTrue(epoch.accept_packet(scan, epoch_id=1))
        self.assertFalse(epoch.accept_packet(replace(scan, entity_id="other"), epoch_id=1))
        with self.assertRaises(TypeError):
            epoch.update_motion(scan, epoch_id=1)

    def test_invalid_inputs_and_configs_do_not_mutate_tracker(self):
        for kwargs in ({"confirmations": 1}, {"confirmations": True},
                       {"max_scan_gap": float("nan")}, {"translation_tolerance": 0}):
            with self.assertRaises(ValueError):
                robotnav.RelocalizationTrackingConfig(**kwargs)
        tracker = robotnav.RelocalizationTracker()
        pose = robotnav.RobotState(5, 6)
        for bad in (replace(result(pose), pose=None),
                    replace(result(pose), accepted=False)):
            with self.assertRaises(ValueError):
                tracker.update(bad, timestamp_ns=0, motion_pose=robotnav.RobotState(0, 0))
        with self.assertRaises(ValueError):
            tracker.update(result(pose), timestamp_ns=0, motion_pose=robotnav.RobotState(float("nan"), 0))
        self.assertEqual(tracker.update(result(pose), timestamp_ns=0,
                         motion_pose=robotnav.RobotState(0, 0)).tracks[0].unique_frames, 1)


if __name__ == "__main__":
    unittest.main()
