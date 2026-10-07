#!/usr/bin/env python3
"""Compare five/seven-state EKFs on one identical, open-loop sensor stream.

Truth is used only to generate measurements and evaluate errors. No map pose
corrections are supplied; unbiased noisy odometry is the bias reference.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import robotnav


def run(*, steps: int = 120, seed: int = 42, bias: float = 0.08,
        random_walk_std: float = 0.0, latency_steps: int = 2,
        drop_probability: float = 0.1, acceleration_bias: float = 0.0,
        acceleration_random_walk_std: float = 0.0,
        odometry_outage_start: int | None = None) -> dict:
    if steps < 1:
        raise ValueError("steps must be positive")
    if odometry_outage_start is not None and not 0 <= odometry_outage_start < steps:
        raise ValueError("odometry_outage_start must be within the evaluated steps")
    timing = robotnav.SensorTiming(latency_steps=latency_steps,
                                  drop_probability=drop_probability)
    initial = robotnav.RobotState(2, 2)
    world = robotnav.SimulationWorld(
        robotnav.WorldConfig(simulation=robotnav.SimulationConfig(dt=0.1), seed=seed),
        [robotnav.WorldEntity("robot", initial)])
    imu_config = robotnav.ImuConfig(
        angular_velocity_bias=bias,
        angular_velocity_bias_random_walk_std=random_walk_std,
        acceleration_bias=acceleration_bias,
        acceleration_bias_random_walk_std=acceleration_random_walk_std,
        angular_velocity_noise_std=0.003, timing=timing)
    sensors = robotnav.SensorSuite(
        ["0" * 30 for _ in range(30)],
        odometries=[robotnav.OdometryConfig(velocity_noise_std=0.01,
                                          heading_noise_std=0.003, timing=timing)],
        imus=[imu_config], seed=seed)
    config = robotnav.EkfConfig(odometry_velocity_std=0.01,
                               odometry_yaw_rate_std=0.03, imu_yaw_rate_std=0.003,
                               gyro_bias_process_std=random_walk_std,
                               acceleration_bias_process_std=acceleration_random_walk_std,
                               velocity_process_std=0.03)
    filters = {"baseline": robotnav.PlanarEkf(initial, config),
               "bias_aware": robotnav.PlanarEkf(
                   initial, replace(config, estimate_gyro_bias=True,
                                    estimate_acceleration_bias=True))}
    trace, packets, withheld = [], [], []
    for index in range(steps):
        observation = world.observation
        batch = sensors.step(observation)
        for packet in batch.ready:
            if (isinstance(packet, robotnav.OdometryMeasurement) and
                    odometry_outage_start is not None and
                    packet.timestamp_ns >= odometry_outage_start * 100_000_000):
                withheld.append(packet.to_dict())
                continue
            packets.append(packet.to_dict())
            for estimator in filters.values():
                estimator.update(packet)
        truth = observation.state("robot")
        row = {"time": observation.clock.time, "truth": asdict(truth),
               "true_gyro_bias": sensors.imu_biases["imu"],
               "true_acceleration_bias": sensors.imu_acceleration_biases["imu"]}
        for name, estimator in filters.items():
            estimate = estimator.estimate_at(round(observation.clock.time * 1e9))
            heading_error = math.atan2(math.sin(estimate.state.theta - truth.theta),
                                       math.cos(estimate.state.theta - truth.theta))
            row[name] = {"state": asdict(estimate.state),
                         "position_error": math.hypot(estimate.state.x - truth.x,
                                                      estimate.state.y - truth.y),
                         "heading_error": heading_error,
                         "velocity_error": estimate.state.velocity - truth.velocity,
                         "gyro_bias": estimate.gyro_bias,
                         "gyro_bias_std": estimate.gyro_bias_std,
                         "acceleration_bias": estimate.acceleration_bias,
                         "acceleration_bias_std": estimate.acceleration_bias_std}
        trace.append(row)
        # Vary turn rate; both estimators see exactly the same world trajectory.
        world.step({"robot": robotnav.ControlCommand(0.6, 0.15 * math.sin(index * 0.08))})
    metrics = {}
    for name, estimator in filters.items():
        metrics[name] = {
            "position_rmse": math.sqrt(sum(row[name]["position_error"] ** 2
                                           for row in trace) / steps),
            "heading_rmse": math.sqrt(sum(row[name]["heading_error"] ** 2
                                          for row in trace) / steps),
            "velocity_rmse": math.sqrt(sum(row[name]["velocity_error"] ** 2
                                           for row in trace) / steps),
            "final_gyro_bias": trace[-1][name]["gyro_bias"],
            "final_gyro_bias_std": trace[-1][name]["gyro_bias_std"],
            "final_acceleration_bias": trace[-1][name]["acceleration_bias"],
            "final_acceleration_bias_std": trace[-1][name]["acceleration_bias_std"],
            "accepted_packets": estimator.accepted_packets,
            "rejected_packets": estimator.rejected_packets,
        }
        # Startup delivery delay can dominate whole-horizon velocity RMSE.
        # Also expose the explicitly defined post-outage interval, without
        # removing startup samples from the overall metrics above.
        if odometry_outage_start is not None:
            outage_rows = trace[odometry_outage_start + latency_steps:]
            metrics[name]["odometry_outage_metrics"] = {
                "samples": len(outage_rows),
                **{f"{field}_rmse": (math.sqrt(sum(row[name][f"{field}_error"] ** 2
                                                   for row in outage_rows) / len(outage_rows))
                                     if outage_rows else None)
                   for field in ("position", "velocity", "heading")},
            }
    # Report only packets delivered before the evaluation horizon; no tail
    # flush changes the final-time comparison. Pending count is explicit.
    return {"schema_version": 1, "seed": seed, "steps": steps,
            "comparison": "identical_observations_no_pose_corrections",
            "imu_config": asdict(imu_config), "ekf_config": asdict(config),
            "ekf_configs": {name: asdict(estimator.config)
                            for name, estimator in filters.items()},
            "pending_packets": batch.pending_count, "metrics": metrics,
            "odometry_outage_start": odometry_outage_start,
            "withheld_packets": withheld, "packets": packets, "trace": trace}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--steps", type=int, default=120)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--bias", type=float, default=0.08)
    parser.add_argument("--random-walk-std", type=float, default=0.0)
    parser.add_argument("--latency-steps", type=int, default=2)
    parser.add_argument("--drop-probability", type=float, default=0.1)
    parser.add_argument("--acceleration-bias", type=float, default=0.0)
    parser.add_argument("--acceleration-random-walk-std", type=float, default=0.0)
    parser.add_argument("--odometry-outage-start", type=int)
    parser.add_argument("--output", default="build/sensor-navigation/imu-bias-benchmark.json")
    args = parser.parse_args(argv)
    output = Path(args.output).expanduser()
    try:
        result = run(**{key: value for key, value in vars(args).items() if key != "output"})
    except ValueError as error:
        parser.error(str(error))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
                      encoding="utf-8")
    print(json.dumps(result["metrics"], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
