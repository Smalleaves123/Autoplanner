"""Regression tests for sensor-only EKF and estimated-pose navigation."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import tempfile
import unittest

import robotnav

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "autoplanner" / "scripts"))
from simulate_sensor_navigation import build_parser, run, _frame_from_lidar  # noqa: E402
from benchmark_imu_bias import run as run_bias_benchmark  # noqa: E402


def odom(sequence: int, *, velocity: float = 1.0) -> robotnav.OdometryMeasurement:
    return robotnav.OdometryMeasurement(
        "odom", "robot", sequence, sequence * 100_000_000,
        sequence * 100_000_000 + 200_000_000,
        999.0, -999.0, 0.0, velocity, 0.1 if sequence else 0.0)


def imu(sequence: int, *, rate: float = 0.0) -> robotnav.ImuMeasurement:
    return robotnav.ImuMeasurement(
        "imu", "robot", sequence, sequence * 100_000_000,
        sequence * 100_000_000 + 100_000_000, 0.0, rate)


class EstimationTests(unittest.TestCase):
    def test_acceleration_bias_recovery_and_outage_prediction(self) -> None:
        config = robotnav.EkfConfig(estimate_acceleration_bias=True,
                                   acceleration_bias_process_std=0.0,
                                   velocity_process_std=0.02,
                                   odometry_velocity_std=0.02)
        initial = robotnav.RobotState(0, 0, velocity=1)
        corrected = robotnav.PlanarEkf(initial, config)
        baseline = robotnav.PlanarEkf(initial, replace(config, estimate_acceleration_bias=False))
        for seq in range(61):
            for packet in (replace(imu(seq), linear_acceleration=0.15), odom(seq)):
                corrected.update(packet)
                baseline.update(packet)
        self.assertAlmostEqual(corrected.estimate.acceleration_bias, 0.15, delta=0.005)
        self.assertLess(corrected.estimate.acceleration_bias_std, 0.02)
        self.assertIsNone(corrected.estimate.gyro_bias_std)
        self.assertEqual(len(corrected.estimate.covariance), 6)
        future = [f.estimate_at(10_000_000_000) for f in (baseline, corrected)]
        self.assertLess(abs(future[1].state.x - 10), abs(future[0].state.x - 10) * 0.1)
        self.assertLess(abs(future[1].state.velocity - 1),
                        abs(future[0].state.velocity - 1) * 0.1)

    def test_acceleration_bias_prediction_jacobian_and_reset(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0, 0), robotnav.EkfConfig(
            estimate_acceleration_bias=True, initial_acceleration_bias=0.2))
        self.assertEqual(ekf.estimate_at(1_000_000_000).state.velocity, 0)
        ekf.update(replace(imu(0), linear_acceleration=0.5))
        future = ekf.estimate_at(1_000_000_000)
        self.assertAlmostEqual(future.state.velocity, 0.3)
        self.assertAlmostEqual(future.state.x, 0.15)
        self.assertLess(future.covariance[3][5], 0)
        self.assertLess(future.covariance[0][5], 0)
        self.assertGreater(future.covariance[5][5], ekf.estimate.covariance[5][5])
        self.assertEqual(future.acceleration_bias, 0.2)
        ekf.reset(robotnav.RobotState(1, 2))
        self.assertEqual(ekf.estimate.acceleration_bias, 0.2)
        self.assertEqual(ekf.estimate.acceleration_bias_std, 0.2)
        self.assertEqual(ekf.accepted_packets, 0)

    def test_acceleration_bias_is_unobservable_from_imu_alone(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0, 0), robotnav.EkfConfig(
            estimate_acceleration_bias=True, acceleration_bias_process_std=0.0))
        for seq in range(20):
            ekf.update(replace(imu(seq), linear_acceleration=0.15))
        self.assertEqual(ekf.estimate.acceleration_bias, 0)
        self.assertAlmostEqual(ekf.estimate.acceleration_bias_std, 0.2)
        self.assertGreater(ekf.estimate.state.velocity, 0.2)

    def test_pose_reference_recovers_acceleration_bias_without_odometry(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0, 0), robotnav.EkfConfig(
            estimate_acceleration_bias=True, acceleration_bias_process_std=0.0,
            position_process_std=0.001, velocity_process_std=0.01))
        for seq in range(61):
            ekf.update(replace(imu(seq), linear_acceleration=0.15))
            if seq % 5 == 0:
                ekf.update(robotnav.PoseMeasurement(
                    "map", "robot", seq, seq * 100_000_000,
                    seq * 100_000_000 + 200_000_000, 0, 0, 0, position_std=0.005))
        self.assertAlmostEqual(ekf.estimate.acceleration_bias, 0.15, delta=0.01)

    def test_both_bias_states_replay_and_covariance(self) -> None:
        initial = robotnav.RobotState(0, 0, velocity=1)
        config = robotnav.EkfConfig(estimate_gyro_bias=True, estimate_acceleration_bias=True,
                                   initial_gyro_bias=-0.01, initial_acceleration_bias=-0.1,
                                   history_seconds=10, max_history_packets=100)
        chronological, delayed = [robotnav.PlanarEkf(initial, config) for _ in range(2)]
        packets = [p for seq in range(14) for p in
                   (replace(imu(seq, rate=0.08), linear_acceleration=0.15), odom(seq))]
        packets.append(robotnav.PoseMeasurement(
            "map", "robot", 1, 500_000_000, 900_000_000, 0.5, 0, 0))
        for p in packets:
            chronological.update(p)
        for p in reversed(packets):
            delayed.update(p)
        self.assertEqual(chronological.estimate, delayed.estimate)
        self.assertEqual(chronological.estimate_at(500_000_000),
                         delayed.estimate_at(500_000_000))
        p = delayed.estimate.covariance
        self.assertEqual(len(p), 7)
        for i in range(7):
            self.assertGreater(p[i][i], 0)
            for j in range(7):
                self.assertTrue(math.isfinite(p[i][j]))
                self.assertAlmostEqual(p[i][j], p[j][i], places=12)
        vector = (1, -2, 3, -4, 5, -6, 7)
        self.assertGreaterEqual(sum(vector[i] * p[i][j] * vector[j]
                                    for i in range(7) for j in range(7)), 0)
        reference = robotnav.PlanarEkf(initial, config)
        limited = robotnav.PlanarEkf(initial, replace(config, max_history_packets=4))
        for packet in packets[:-1]:
            reference.update(packet)
            limited.update(packet)
        self.assertEqual(limited.estimate, reference.estimate)
        delayed.reset(initial)
        self.assertEqual(delayed.estimate.gyro_bias, -0.01)
        self.assertEqual(delayed.estimate.acceleration_bias, -0.1)

    def test_acceleration_bias_benchmark_with_odometry_outage(self) -> None:
        result = run_bias_benchmark(steps=90, bias=0, acceleration_bias=0.15,
                                    odometry_outage_start=60)
        self.assertGreater(len(result["withheld_packets"]), 0)
        metrics = result["metrics"]
        self.assertEqual(metrics["baseline"]["accepted_packets"],
                         metrics["bias_aware"]["accepted_packets"])
        self.assertLess(metrics["bias_aware"]["odometry_outage_metrics"]["velocity_rmse"],
                        metrics["baseline"]["odometry_outage_metrics"]["velocity_rmse"] * 0.2)
        self.assertAlmostEqual(metrics["bias_aware"]["final_acceleration_bias"],
                               0.15, delta=0.03)

    def test_navigation_bias_modes_have_compatible_dimensions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            for gyro, acceleration, dimension in ((False, False, 5), (True, False, 6),
                                                  (False, True, 6), (True, True, 7)):
                args = build_parser().parse_args([
                    "--steps", "4", "--beams", "11", "--goal", "8", "2",
                    "--localization", "dead_reckoning", "--latency-steps", "0",
                    "--drop-probability", "0", "--imu-acceleration-bias", "0.15",
                    "--estimate-gyro-bias" if gyro else "--no-estimate-gyro-bias",
                    "--estimate-acceleration-bias" if acceleration else
                    "--no-estimate-acceleration-bias",
                    "--output", str(Path(temporary) / "trace.json"),
                ])
                row = run(args)["trace"][-1]
                self.assertEqual(len(row["estimate_covariance"]), dimension)
                self.assertEqual(row["gyro_bias_std"] is not None, gyro)
                self.assertEqual(row["acceleration_bias_std"] is not None, acceleration)
                self.assertEqual(row["estimated_acceleration_bias"] is not None, acceleration)
                self.assertEqual(row["true_acceleration_bias"], 0.15)


    def test_bias_benchmark_is_repeatable_with_equal_packet_counts(self) -> None:
        first = run_bias_benchmark(steps=25, random_walk_std=0.002)
        self.assertEqual(first, run_bias_benchmark(steps=25, random_walk_std=0.002))
        self.assertEqual(first["metrics"]["baseline"]["accepted_packets"],
                         first["metrics"]["bias_aware"]["accepted_packets"])
        self.assertEqual(first["metrics"]["bias_aware"]["accepted_packets"],
                         len(first["packets"]))
        self.assertLess(first["metrics"]["bias_aware"]["position_rmse"],
                        first["metrics"]["baseline"]["position_rmse"])

    def test_bias_estimation_reduces_drift_on_identical_sensor_stream(self) -> None:
        initial = robotnav.RobotState(0.0, 0.0, velocity=1.0)
        config = robotnav.EkfConfig(imu_yaw_rate_std=0.005,
                                   odometry_yaw_rate_std=0.04,
                                   gyro_bias_process_std=0.0)
        baseline = robotnav.PlanarEkf(initial, config)
        corrected = robotnav.PlanarEkf(initial, replace(config, estimate_gyro_bias=True))
        for seq in range(101):
            for packet in (imu(seq, rate=0.08), odom(seq)):
                baseline.update(packet)
                corrected.update(packet)
        self.assertAlmostEqual(corrected.estimate.gyro_bias, 0.08, delta=0.002)
        self.assertLess(corrected.estimate.gyro_bias_std, 0.01)
        truth_x = 10.0
        errors = [math.hypot(f.estimate.state.x - truth_x, f.estimate.state.y)
                  for f in (baseline, corrected)]
        self.assertLess(errors[1], errors[0] * 0.1)
        self.assertLess(abs(corrected.estimate.state.theta),
                        abs(baseline.estimate.state.theta) * 0.1)
        self.assertEqual(len(baseline.estimate.covariance), 5)
        self.assertIsNone(baseline.estimate.gyro_bias_std)
        p = corrected.estimate.covariance
        for i in range(6):
            self.assertGreater(p[i][i], 0)
            for j in range(6):
                self.assertTrue(math.isfinite(p[i][j]))
                self.assertAlmostEqual(p[i][j], p[j][i], places=12)
        for vector in ((1, -2, 3, -4, 5, -6), (0, 0, 0, 0, 1, -1)):
            self.assertGreaterEqual(sum(vector[i] * p[i][j] * vector[j]
                                        for i in range(6) for j in range(6)), 0)

    def test_gyro_only_cannot_observe_bias_without_reference(self) -> None:
        config = robotnav.EkfConfig(estimate_gyro_bias=True,
                                   initial_yaw_rate_std=0.1, initial_gyro_bias_std=0.1,
                                   gyro_bias_process_std=0.0)
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0, 0), config)
        for seq in range(25):
            ekf.update(imu(seq, rate=0.08))
        self.assertGreater(ekf.estimate.gyro_bias_std, 0.06)
        self.assertGreater(abs(ekf.estimate.gyro_bias - 0.08), 0.02)

    def test_pose_reference_recovers_bias_without_odometry(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0, 0), robotnav.EkfConfig(
            estimate_gyro_bias=True, gyro_bias_process_std=0.0,
            imu_yaw_rate_std=0.003, heading_process_std=0.001))
        for seq in range(61):
            ekf.update(imu(seq, rate=0.08))
            if seq % 5 == 0:
                ekf.update(robotnav.PoseMeasurement(
                    "map", "robot", seq, seq * 100_000_000,
                    seq * 100_000_000 + 200_000_000, 0, 0, 0,
                    heading_std=0.005))
        self.assertAlmostEqual(ekf.estimate.gyro_bias, 0.08, delta=0.005)

    def test_bias_replay_pose_correction_prediction_and_reset(self) -> None:
        config = robotnav.EkfConfig(estimate_gyro_bias=True, initial_gyro_bias=-0.01,
                                   history_seconds=10, max_history_packets=100)
        initial = robotnav.RobotState(0, 0, velocity=1)
        filters = [robotnav.PlanarEkf(initial, config) for _ in range(2)]
        packets = [p for seq in range(12) for p in (imu(seq, rate=0.08), odom(seq))]
        packets.append(robotnav.PoseMeasurement(
            "map", "robot", 1, 500_000_000, 900_000_000, 0.5, 0, 0))
        for p in packets:
            filters[0].update(p)
        for p in reversed(packets):
            filters[1].update(p)
        self.assertEqual(filters[0].estimate, filters[1].estimate)
        self.assertEqual(filters[0].estimate_at(500_000_000),
                         filters[1].estimate_at(500_000_000))
        before = filters[0].estimate
        future = filters[0].estimate_at(2_000_000_000)
        self.assertEqual(before.gyro_bias, future.gyro_bias)
        self.assertGreater(future.covariance[5][5], before.covariance[5][5])
        self.assertEqual(filters[0].estimate, before)
        filters[0].reset(initial)
        self.assertEqual(filters[0].estimate.gyro_bias, -0.01)
        self.assertEqual(filters[0].estimate.gyro_bias_std, 0.1)
        self.assertEqual(filters[0].accepted_packets, 0)
        # Exercise checkpoint commitment with the augmented covariance.
        limited = robotnav.PlanarEkf(initial, replace(config, max_history_packets=4))
        reference = robotnav.PlanarEkf(initial, config)
        for p in packets[:-1]:
            limited.update(p)
            reference.update(p)
        self.assertEqual(limited.estimate, reference.estimate)
        self.assertLessEqual(len(limited._events), 4)
        self.assertEqual(len(limited.estimate.covariance), 6)

    def test_straight_motion_ignores_truth_frame_displacement(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(2.0, 3.0, velocity=1.0))
        for sequence in range(11):
            ekf.update(odom(sequence))
            ekf.update(imu(sequence))
        estimate = ekf.estimate
        self.assertAlmostEqual(estimate.state.x, 3.0)
        self.assertAlmostEqual(estimate.state.y, 3.0)
        self.assertAlmostEqual(estimate.state.theta, 0.0)

    def test_late_packets_produce_same_result_as_event_order(self) -> None:
        packets = [packet for seq in range(8) for packet in (imu(seq, rate=0.2), odom(seq))]
        chronological = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0, velocity=1.0))
        delayed = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0, velocity=1.0))
        for packet in packets:
            chronological.update(packet)
        for packet in reversed(packets):
            delayed.update(packet)
        self.assertEqual(delayed.estimate, chronological.estimate)
        self.assertEqual(delayed.estimate_at(300_000_000),
                         chronological.estimate_at(300_000_000))
        before = delayed.estimate
        future = delayed.estimate_at(900_000_000)
        self.assertEqual(delayed.estimate, before)
        self.assertGreater(future.state.x, before.state.x)

    def test_imu_fusion_and_covariance_are_finite_symmetric_positive(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0), robotnav.EkfConfig(
            imu_yaw_rate_std=0.005, odometry_yaw_rate_std=1.0))
        for seq in range(30):
            ekf.update(replace(odom(seq), delta_theta=0.05))
            ekf.update(imu(seq, rate=0.1))
        self.assertAlmostEqual(ekf.estimate.angular_velocity, 0.1, delta=0.001)
        p = ekf.estimate.covariance
        for i in range(5):
            self.assertGreater(p[i][i], 0.0)
            for j in range(5):
                self.assertTrue(math.isfinite(p[i][j]))
                self.assertAlmostEqual(p[i][j], p[j][i], places=12)
        for vector in ((1, 1, 1, 1, 1), (1, -2, 3, -4, 5), (0, 0, 1, 0, -1)):
            quadratic = sum(vector[i] * p[i][j] * vector[j]
                            for i in range(5) for j in range(5))
            self.assertGreaterEqual(quadratic, 0.0)

    def test_duplicate_expired_wrong_entity_and_invalid_packets(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0),
                                robotnav.EkfConfig(history_seconds=0.2))
        self.assertTrue(ekf.update(odom(0)))
        self.assertFalse(ekf.update(odom(0)))
        self.assertFalse(ekf.update(replace(odom(1), entity_id="other")))
        ekf.update(odom(10))
        self.assertFalse(ekf.update(odom(1)))
        before = ekf.estimate
        with self.assertRaises(ValueError):
            ekf.update(replace(imu(11), angular_velocity=float("nan")))
        self.assertEqual(before, ekf.estimate)
        with self.assertRaises(ValueError):
            ekf.estimate_at(0)
        ekf.reset(robotnav.RobotState(1.0, 2.0))
        self.assertEqual(ekf.accepted_packets, 0)
        self.assertEqual(ekf.rejected_packets, 0)
        self.assertEqual(ekf.estimate.state, robotnav.RobotState(1.0, 2.0))

    def test_history_packet_limit_and_heading_wrap(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0, math.pi - 0.01),
                                robotnav.EkfConfig(max_history_packets=4))
        for seq in range(12):
            ekf.update(imu(seq, rate=0.3))
        self.assertLessEqual(len(ekf._events), 4)
        self.assertGreater(ekf.oldest_timestamp_ns, 0)
        self.assertTrue(-math.pi <= ekf.estimate.state.theta <= math.pi)

    def test_acceleration_prediction_grows_uncertainty_without_mutation(self) -> None:
        ekf = robotnav.PlanarEkf(robotnav.RobotState(0.0, 0.0))
        ekf.update(replace(imu(0), linear_acceleration=1.0))
        before = ekf.estimate
        future = ekf.estimate_at(1_000_000_000)
        self.assertAlmostEqual(future.state.x, 0.5)
        self.assertAlmostEqual(future.state.velocity, 1.0)
        self.assertGreater(future.covariance[0][0], before.covariance[0][0])
        self.assertEqual(ekf.estimate, before)

    def test_lidar_projection_uses_estimate_not_embedded_truth(self) -> None:
        scan = robotnav.LidarScan(
            "lidar", "robot", 0, 100, 200, "map",
            robotnav.RobotState(999.0, 999.0), (0.0,), (2.0,), (True,), ("dynamic",))
        pose = robotnav.RobotState(1.0, 2.0, math.pi / 2)
        frame = _frame_from_lidar(scan, pose)
        self.assertAlmostEqual(frame.points[0].x, 1.0)
        self.assertAlmostEqual(frame.points[0].y, 4.0)
        self.assertEqual(frame.timestamp_ns, 100)
        self.assertEqual(frame.arrival_timestamp_ns, 200)
        self.assertEqual(frame.points[0].kind, "dynamic")

    def test_invalid_config_rejected(self) -> None:
        for kwargs in ({"history_seconds": 0}, {"max_history_packets": 0},
                       {"imu_yaw_rate_std": float("nan")},
                       {"estimate_gyro_bias": 1}, {"initial_gyro_bias": float("inf")},
                       {"gyro_bias_process_std": -1}, {"initial_gyro_bias_std": 0},
                       {"estimate_acceleration_bias": 1},
                       {"initial_acceleration_bias": float("nan")},
                       {"initial_acceleration_bias_std": 0},
                       {"acceleration_bias_process_std": -1}):
            with self.assertRaises(ValueError):
                robotnav.EkfConfig(**kwargs)

    def test_noisy_delayed_navigation_is_repeatable_and_moves(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            args = build_parser().parse_args([
                "--steps", "20", "--beams", "31", "--goal", "8", "2",
                "--latency-steps", "2", "--drop-probability", "0.15",
                "--timestamp-jitter-ns", "5000000",
                "--output", str(Path(temporary) / "trace.json"),
            ])
            first, second = run(args), run(args)
        self.assertEqual(first, second)
        self.assertGreater(first["accepted_motion_packets"], 0)
        self.assertGreater(first["dropped_packets"], 0)
        self.assertGreater(first["trace"][-1]["robot"]["x"], 2.5)
        self.assertLess(first["position_rmse"], 0.25)
        self.assertTrue(first["trace"][0]["safe_stop"])
        self.assertTrue(first["ekf_config"]["estimate_gyro_bias"])
        self.assertEqual(len(first["trace"][-1]["estimate_covariance"]), 6)
        self.assertIsNotNone(first["trace"][-1]["gyro_bias_std"])

    def test_total_sensor_loss_stops_robot(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            args = build_parser().parse_args([
                "--steps", "5", "--drop-probability", "1",
                "--no-estimate-gyro-bias",
                "--output", str(Path(temporary) / "trace.json"),
            ])
            result = run(args)
        self.assertEqual(result["accepted_motion_packets"], 0)
        self.assertEqual(result["safe_stop_steps"], 5)
        self.assertEqual(len(result["trace"][-1]["estimate_covariance"]), 5)
        self.assertIsNone(result["trace"][-1]["gyro_bias_std"])
        self.assertTrue(all(row["robot"]["x"] == 2.5 and
                            row["robot"]["y"] == 2.5 for row in result["trace"]))


if __name__ == "__main__":
    unittest.main()
