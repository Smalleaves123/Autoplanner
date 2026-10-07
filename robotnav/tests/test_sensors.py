from __future__ import annotations

import unittest
import math

import robotnav


MAP = "autoplanner/data/maps/simple_50x50.txt"


def _world() -> robotnav.SimulationWorld:
    return robotnav.SimulationWorld(
        robotnav.WorldConfig(
            simulation=robotnav.SimulationConfig(dt=0.1), seed=9),
        [robotnav.WorldEntity("robot", robotnav.RobotState(2.0, 2.0))],
    )


class SensorSuiteTests(unittest.TestCase):
    def test_constant_gyro_bias_and_validation(self) -> None:
        suite = robotnav.SensorSuite(MAP, imus=[robotnav.ImuConfig(
            angular_velocity_bias=-0.08, acceleration_bias=-0.2)])
        packet = suite.step(_world().observation).ready[0]
        self.assertEqual(packet.angular_velocity, -0.08)
        self.assertEqual(suite.imu_biases, {"imu": -0.08})
        self.assertEqual(packet.linear_acceleration, -0.2)
        self.assertEqual(suite.imu_acceleration_biases, {"imu": -0.2})
        self.assertNotIn("angular_velocity_bias", packet.to_dict())
        for kwargs in ({"angular_velocity_bias": float("nan")},
                       {"angular_velocity_bias_random_walk_std": -0.01},
                       {"angular_velocity_bias_random_walk_std": float("inf")},
                       {"angular_velocity_noise_std": float("nan")},
                       {"acceleration_bias": float("inf")},
                       {"acceleration_bias_random_walk_std": -1},
                       {"acceleration_bias_random_walk_std": float("nan")}):
            with self.assertRaises(ValueError):
                robotnav.ImuConfig(**kwargs)

    def test_bias_walk_uses_physical_time_through_drop_and_latency(self) -> None:
        config = robotnav.ImuConfig(angular_velocity_bias=0.08,
                                   angular_velocity_bias_random_walk_std=0.02,
                                   acceleration_bias=0.15,
                                   acceleration_bias_random_walk_std=0.01)
        from dataclasses import replace
        full = robotnav.SensorSuite(MAP, imus=[config], seed=42)
        lost = robotnav.SensorSuite(MAP, imus=[replace(config, timing=
            robotnav.SensorTiming(drop_probability=1.0))], seed=42)
        delayed = robotnav.SensorSuite(MAP, imus=[replace(config, timing=
            robotnav.SensorTiming(latency_steps=2, timestamp_jitter_ns=1000))], seed=42)
        world = _world()
        observations, packets, biases, acceleration_biases = [], [], [], []
        for _ in range(12):
            observations.append(world.observation)
            packets.append(full.step(world.observation).ready)
            lost.step(world.observation)
            delayed_batch = delayed.step(world.observation)
            biases.append(full.imu_biases["imu"])
            acceleration_biases.append(full.imu_acceleration_biases["imu"])
            self.assertEqual(full.imu_biases, lost.imu_biases)
            self.assertEqual(full.imu_biases, delayed.imu_biases)
            self.assertEqual(full.imu_acceleration_biases, lost.imu_acceleration_biases)
            self.assertEqual(full.imu_acceleration_biases, delayed.imu_acceleration_biases)
            for packet in delayed_batch.ready:
                self.assertEqual(packet.angular_velocity, biases[packet.sequence])
                self.assertEqual(packet.linear_acceleration, acceleration_biases[packet.sequence])
            world.step({})
        self.assertEqual(biases[0], 0.08)
        self.assertGreater(len(set(biases)), 1)
        self.assertGreater(len(set(acceleration_biases)), 1)
        full.reset()
        self.assertEqual(full.imu_biases, {"imu": 0.08})
        self.assertEqual(full.imu_acceleration_biases, {"imu": 0.15})
        for observation, expected in zip(observations, packets):
            self.assertEqual(full.step(observation).ready, expected)
        before = full.imu_biases
        full.drain(world.observation)
        self.assertEqual(full.imu_biases, before)

    def test_bias_walk_scales_with_square_root_of_elapsed_time(self) -> None:
        config = robotnav.ImuConfig(angular_velocity_bias_random_walk_std=0.02,
                                   acceleration_bias_random_walk_std=0.03)
        suites = [robotnav.SensorSuite(MAP, imus=[config], seed=4) for _ in range(2)]
        for suite in suites:
            suite.step(_world().observation)
        for suite, elapsed in zip(suites, (0.1, 1.0)):
            suite.step(robotnav.WorldObservation(
                robotnav.SimulationClock(1, elapsed, 0.1),
                (robotnav.EntityObservation("robot", robotnav.RobotState(2, 2)),)))
        self.assertAlmostEqual(suites[1].imu_biases["imu"],
                               suites[0].imu_biases["imu"] * math.sqrt(10))
        self.assertAlmostEqual(suites[1].imu_acceleration_biases["imu"],
                               suites[0].imu_acceleration_biases["imu"] * math.sqrt(10))

    def test_motion_intervals_and_heading_wrap(self) -> None:
        suite = robotnav.SensorSuite(
            ["000" for _ in range(3)],
            odometries=[robotnav.OdometryConfig()],
            imus=[robotnav.ImuConfig()],
        )

        def observation(index: int, heading: float) -> robotnav.WorldObservation:
            return robotnav.WorldObservation(
                robotnav.SimulationClock(index, index * 0.1, 0.1),
                (robotnav.EntityObservation("robot", robotnav.RobotState(
                    1.0, 1.0, heading)),),
            )

        first = suite.step(observation(0, math.pi - 0.01))
        self.assertEqual(first.ready[0].sample_dt, 0.0)
        second = suite.step(observation(1, -math.pi + 0.01))
        odom = next(item for item in second.ready
                    if isinstance(item, robotnav.OdometryMeasurement))
        imu = next(item for item in second.ready
                   if isinstance(item, robotnav.ImuMeasurement))
        self.assertAlmostEqual(odom.delta_theta, 0.02)
        self.assertAlmostEqual(odom.sample_dt, 0.1)
        self.assertAlmostEqual(imu.angular_velocity, 0.2)

    def test_all_sensor_types_emit_timestamped_packets(self) -> None:
        world = _world()
        suite = robotnav.SensorSuite(
            MAP,
            lidars=[robotnav.Lidar2DConfig(num_beams=7, max_range=5.0)],
            odometries=[robotnav.OdometryConfig()],
            imus=[robotnav.ImuConfig()],
            depth_cameras=[robotnav.DepthCameraConfig(width=4, height=3)],
            seed=4,
        )
        batch = suite.step(world.observation)
        self.assertEqual(len(batch.ready), 4)
        self.assertEqual({type(packet).__name__ for packet in batch.ready}, {
            "LidarScan", "OdometryMeasurement", "ImuMeasurement", "DepthImage",
        })
        lidar = next(packet for packet in batch.ready
                      if isinstance(packet, robotnav.LidarScan))
        depth = next(packet for packet in batch.ready
                      if isinstance(packet, robotnav.DepthImage))
        self.assertEqual(len(lidar.ranges), 7)
        self.assertEqual(len(depth.depths), 12)
        self.assertTrue(all(value >= 0.0 for value in lidar.ranges))

    def test_latency_and_drop_are_deterministic(self) -> None:
        world = _world()
        suite = robotnav.SensorSuite(
            MAP,
            odometries=[robotnav.OdometryConfig(
                timing=robotnav.SensorTiming(latency_steps=1))],
            imus=[robotnav.ImuConfig(
                sensor_id="drop_imu",
                timing=robotnav.SensorTiming(drop_probability=1.0))],
            seed=12,
        )
        first = suite.step(world.observation)
        self.assertEqual(first.ready, ())
        self.assertEqual(first.dropped, ("drop_imu",))
        self.assertEqual(first.pending_count, 1)
        second = suite.step(world.step({}))
        self.assertEqual(second.dropped, ("drop_imu",))
        self.assertEqual(len(second.ready), 1)
        self.assertIsInstance(second.ready[0], robotnav.OdometryMeasurement)

    def test_drain_releases_tail_without_creating_an_extra_sample(self) -> None:
        world = _world()
        suite = robotnav.SensorSuite(
            MAP,
            lidars=[robotnav.Lidar2DConfig(
                num_beams=5,
                timing=robotnav.SensorTiming(latency_steps=2),
            )],
        )
        first = suite.step(world.observation)
        self.assertEqual(first.pending_count, 1)
        world.step({})
        middle = suite.drain(world.observation)
        self.assertEqual(middle.ready, ())
        self.assertEqual(middle.pending_count, 1)
        world.step({})
        tail = suite.drain(world.observation)
        self.assertEqual(len(tail.ready), 1)
        self.assertEqual(tail.ready[0].sequence, 0)
        self.assertEqual(tail.pending_count, 0)

    def test_same_seed_produces_same_noisy_scan(self) -> None:
        config = robotnav.Lidar2DConfig(
            num_beams=5, max_range=4.0, range_noise_std=0.05,
            timing=robotnav.SensorTiming())
        first = robotnav.SensorSuite(MAP, lidars=[config], seed=42)
        second = robotnav.SensorSuite(MAP, lidars=[config], seed=42)
        observation = _world().observation
        self.assertEqual(
            first.step(observation).ready[0].to_dict(),
            second.step(observation).ready[0].to_dict(),
        )

    def test_lidar_labels_other_world_entities_as_dynamic(self) -> None:
        world = robotnav.SimulationWorld(
            robotnav.WorldConfig(
                simulation=robotnav.SimulationConfig(dt=0.1)),
            [
                robotnav.WorldEntity("robot", robotnav.RobotState(2.5, 2.5)),
                robotnav.WorldEntity("target", robotnav.RobotState(5.5, 2.5),
                                     radius=0.45),
            ],
        )
        suite = robotnav.SensorSuite(
            ["0000000000" for _ in range(10)],
            lidars=[robotnav.Lidar2DConfig(
                num_beams=31, field_of_view=3.14, max_range=6.0,
                ray_step=0.05)],
        )
        scan = suite.step(world.observation).ready[0]
        self.assertIsInstance(scan, robotnav.LidarScan)
        self.assertIn("dynamic", scan.kinds)
        frame = scan.to_perception_frame()
        self.assertIn("dynamic", {point["kind"] for point in frame["points"]})


if __name__ == "__main__":
    unittest.main()
