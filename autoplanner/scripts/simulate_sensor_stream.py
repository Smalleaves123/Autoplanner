#!/usr/bin/env python3
"""Generate a deterministic multi-sensor stream from the RobotNav world API."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import robotnav


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--output", default="autoplanner/results/sensors/sensor_stream.json")
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--drop-probability", type=float, default=0.0)
    parser.add_argument("--latency-steps", type=int, default=0)
    parser.add_argument("--imu-gyro-bias", type=float, default=0.0)
    parser.add_argument("--imu-gyro-bias-random-walk", type=float, default=0.0)
    parser.add_argument("--imu-acceleration-bias", type=float, default=0.0)
    parser.add_argument("--imu-acceleration-bias-random-walk", type=float, default=0.0)
    args = parser.parse_args(argv)
    if args.steps <= 0:
        raise SystemExit("--steps must be positive")
    timing = robotnav.SensorTiming(
        latency_steps=args.latency_steps,
        drop_probability=args.drop_probability,
    )
    world = robotnav.SimulationWorld(
        robotnav.WorldConfig(
            simulation=robotnav.SimulationConfig(dt=0.1),
            seed=args.seed,
        ),
        [robotnav.WorldEntity("robot", robotnav.RobotState(2.0, 2.0))],
    )
    sensors = robotnav.SensorSuite(
        args.map,
        lidars=[robotnav.Lidar2DConfig(
            sensor_id="lidar", num_beams=91, max_range=15.0,
            timing=timing)],
        odometries=[robotnav.OdometryConfig(
            sensor_id="odom", timing=timing)],
        imus=[robotnav.ImuConfig(
            sensor_id="imu", timing=timing,
            angular_velocity_bias=args.imu_gyro_bias,
            angular_velocity_bias_random_walk_std=args.imu_gyro_bias_random_walk,
            acceleration_bias=args.imu_acceleration_bias,
            acceleration_bias_random_walk_std=args.imu_acceleration_bias_random_walk)],
        depth_cameras=[robotnav.DepthCameraConfig(
            sensor_id="depth", width=32, height=24, timing=timing)],
        seed=args.seed,
    )
    packets: list[dict] = []
    dropped: list[dict] = []

    def record(batch: robotnav.SensorBatch) -> None:
        packets.extend(
            packet.to_dict() | {"world_step": batch.clock.step_index}
            for packet in batch.ready
        )
        dropped.extend({
            "sensor_id": sensor_id,
            "world_step": batch.clock.step_index,
        } for sensor_id in batch.dropped)

    for _ in range(args.steps):
        batch = sensors.step(world.observation)
        record(batch)
        world.step({"robot": robotnav.ControlCommand(0.6, 0.0)})

    # Flush delayed packets without creating additional sensor samples.
    for _ in range(args.latency_steps + 1):
        world.step({})
        batch = sensors.drain(world.observation)
        record(batch)
        if batch.pending_count == 0:
            break

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({
        "schema_version": 1,
        "map": str(Path(args.map).expanduser()),
        "seed": args.seed,
        "steps": args.steps,
        "timing": {
            "latency_steps": args.latency_steps,
            "drop_probability": args.drop_probability,
        },
        "imu": {
            "angular_velocity_bias": args.imu_gyro_bias,
            "angular_velocity_bias_random_walk_std": args.imu_gyro_bias_random_walk,
            "acceleration_bias": args.imu_acceleration_bias,
            "acceleration_bias_random_walk_std": args.imu_acceleration_bias_random_walk,
        },
        "packets": packets,
        "dropped": dropped,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
