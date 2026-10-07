#!/usr/bin/env python3
"""Run a deterministic sensor -> perception -> planning -> control loop.

The demo intentionally stays ROS-free.  A :class:`robotnav.SimulationWorld`
produces robot and moving-obstacle states, :class:`robotnav.SensorSuite`
delivers timestamped LiDAR packets, and the existing perception pipeline
integrates them into an occupancy map, tracks dynamic returns, and replans.
The resulting control command is fed back into the same world.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from typing import Any

import robotnav

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(SCRIPT_DIR))

from perception_mapping import (  # noqa: E402
    PerceptionMappingPipeline,
    SensorFrame,
    SensorPoint,
    load_grid,
)


def _frame_from_lidar(scan: robotnav.LidarScan,
                      pose: robotnav.RobotState) -> SensorFrame:
    """Project rays from an estimated capture-time pose, never scan.pose."""
    frame = scan.to_perception_frame(pose=pose)
    points = tuple(SensorPoint(**point) for point in frame["points"])
    return SensorFrame(
        scan.sequence,
        pose.x,
        pose.y,
        points,
        pose.theta,
        scan.timestamp_ns,
        scan.arrival_timestamp_ns,
    )


def _angle_error(target: float, heading: float) -> float:
    return math.atan2(math.sin(target - heading), math.cos(target - heading))


def _control_for_path(
    state: robotnav.RobotState,
    path: tuple[tuple[int, int], ...],
    *,
    max_velocity: float,
    max_steering: float,
) -> robotnav.ControlCommand:
    if len(path) < 2:
        return robotnav.ControlCommand(0.0, 0.0)
    # Skip cells already reached and target the next cell center.
    target = path[1]
    tx, ty = target[0] + 0.5, target[1] + 0.5
    dx, dy = tx - state.x, ty - state.y
    distance = math.hypot(dx, dy)
    desired = math.atan2(dy, dx)
    error = _angle_error(desired, state.theta)
    steering = max(-max_steering, min(max_steering, 1.8 * error))
    velocity = min(max_velocity, max(0.0, 1.2 * distance))
    if abs(error) > 1.2:
        velocity *= 0.25
    return robotnav.ControlCommand(velocity, steering)


def run(args: argparse.Namespace) -> dict[str, Any]:
    map_path = Path(args.map).expanduser()
    if not map_path.is_absolute():
        map_path = (REPO_ROOT / map_path).resolve()
    grid = load_grid(map_path)
    start = tuple(args.start)
    goal = tuple(args.goal)
    obstacle = tuple(args.obstacle)
    dt = args.dt
    initial = robotnav.RobotState(start[0] + 0.5, start[1] + 0.5)
    world = robotnav.SimulationWorld(
        robotnav.WorldConfig(
            simulation=robotnav.SimulationConfig(dt=dt), seed=args.seed),
        [
            robotnav.WorldEntity(
                "robot", initial,
                radius=0.25),
            robotnav.WorldEntity(
                "moving_obstacle",
                robotnav.RobotState(obstacle[0] + 0.5, obstacle[1] + 0.5,
                                    args.obstacle_heading),
                radius=args.obstacle_radius),
        ],
    )
    timing = robotnav.SensorTiming(
        latency_steps=args.latency_steps,
        drop_probability=args.drop_probability,
        timestamp_jitter_ns=args.timestamp_jitter_ns,
    )
    sensors = robotnav.SensorSuite(
        map_path,
        lidars=[robotnav.Lidar2DConfig(
            num_beams=args.beams, max_range=args.sensor_range,
            ray_step=args.ray_step, range_noise_std=args.sensor_noise,
            timing=timing)],
        odometries=[robotnav.OdometryConfig(
            velocity_noise_std=args.odom_velocity_noise,
            heading_noise_std=args.odom_heading_noise, timing=timing)],
        imus=[robotnav.ImuConfig(
            acceleration_noise_std=args.imu_acceleration_noise,
            angular_velocity_noise_std=args.imu_yaw_rate_noise, timing=timing,
            angular_velocity_bias=args.imu_gyro_bias,
            angular_velocity_bias_random_walk_std=args.imu_gyro_bias_random_walk,
            acceleration_bias=args.imu_acceleration_bias,
            acceleration_bias_random_walk_std=args.imu_acceleration_bias_random_walk)],
        seed=args.seed,
    )
    offset = args.initial_estimate_offset
    estimated_initial = robotnav.RobotState(
        initial.x + offset[0], initial.y + offset[1], initial.theta + offset[2])
    estimator = robotnav.PlanarEkf(estimated_initial, robotnav.EkfConfig(
        odometry_velocity_std=max(1e-6, args.odom_velocity_noise),
        odometry_yaw_rate_std=max(1e-6, args.odom_heading_noise / dt),
        imu_yaw_rate_std=max(1e-6, args.imu_yaw_rate_noise),
        estimate_gyro_bias=args.estimate_gyro_bias,
        gyro_bias_process_std=args.ekf_gyro_bias_process_std,
        estimate_acceleration_bias=args.estimate_acceleration_bias,
        acceleration_bias_process_std=args.ekf_acceleration_bias_process_std,
    ))
    matcher = (robotnav.GridScanMatcher(map_path, robotnav.ScanMatcherConfig(
        translation_window=args.scan_match_translation_window,
        rotation_window=args.scan_match_rotation_window))
        if args.localization == "scan_matching" else None)
    pipeline = PerceptionMappingPipeline(
        grid, start, goal, unknown_policy="occupied",
        initial_map_known=True, robot_radius=0.25,
    )
    trace: list[dict[str, Any]] = []
    total_dropped = 0
    lidar_packets = 0
    replans = 0
    stale_scans = 0
    skipped_scans = 0
    last_scan_sequence = -1
    last_scan_timestamp_ns: int | None = None
    safe_stop_steps = 0
    match_attempts = 0
    for _ in range(args.steps):
        observation = world.observation
        batch = sensors.step(observation)
        total_dropped += len(batch.dropped)
        latest = None
        match_records: list[dict[str, Any]] = []
        # Fuse all motion packets first; arrival ordering in a SensorBatch
        # must not determine which pose a scan uses at the same timestamp.
        for packet in batch.ready:
            if isinstance(packet, (robotnav.OdometryMeasurement, robotnav.ImuMeasurement)):
                estimator.update(packet)
        now_ns = int(round(observation.clock.time * 1e9))
        estimate = estimator.estimate_at(now_ns)
        state = estimate.state
        for packet in batch.ready:
            if not isinstance(packet, robotnav.LidarScan):
                continue
            lidar_packets += 1
            if packet.sequence <= last_scan_sequence:
                stale_scans += 1
                continue
            try:
                capture_pose = estimator.estimate_at(packet.timestamp_ns).state
            except ValueError:
                skipped_scans += 1
                continue
            if matcher is not None and packet.sequence % args.scan_match_every == 0:
                match_attempts += 1
                match = matcher.match(packet, capture_pose)
                fused = False
                if match.accepted:
                    fused = estimator.update(robotnav.PoseMeasurement(
                        packet.sensor_id + "_map", packet.entity_id, packet.sequence,
                        packet.timestamp_ns, packet.arrival_timestamp_ns,
                        match.pose.x, match.pose.y, match.pose.theta,
                        args.scan_match_position_std, args.scan_match_heading_std))
                    if fused:
                        capture_pose = estimator.estimate_at(packet.timestamp_ns).state
                match_records.append({
                    **asdict(match), "sequence": packet.sequence,
                    "timestamp_ns": packet.timestamp_ns, "fused": fused,
                })
            # A delayed correction replays EKF history; use the corrected
            # current estimate for the planner's robot cell as well.
            estimate = estimator.estimate_at(now_ns)
            state = estimate.state
            latest = pipeline.step(
                _frame_from_lidar(packet, capture_pose),
                (math.floor(state.x), math.floor(state.y)),
            )
            last_scan_sequence = packet.sequence
            last_scan_timestamp_ns = packet.timestamp_ns
            replans += int(latest.replanned)
        estimate_age = max(0.0, (now_ns - estimator.last_motion_timestamp_ns) * 1e-9)
        localization_stale = (estimator.accepted_motion_packets == 0 or
                              estimate_age > args.max_estimate_age)
        scan_age = (float("inf") if last_scan_timestamp_ns is None else
                    max(0.0, (now_ns - last_scan_timestamp_ns) * 1e-9))
        safe_stop = localization_stale or scan_age > args.max_scan_age
        safe_stop_steps += int(safe_stop)
        truth = observation.state("robot")  # evaluation only
        goal_reached = math.hypot(
            truth.x - (goal[0] + 0.5), truth.y - (goal[1] + 0.5)
        ) <= args.goal_tolerance
        command = _control_for_path(
            state, pipeline.path, max_velocity=args.max_velocity,
            max_steering=args.max_steering,
        )
        if safe_stop or math.hypot(
                state.x - (goal[0] + 0.5), state.y - (goal[1] + 0.5)
        ) <= args.goal_tolerance:
            command = robotnav.ControlCommand(0.0, 0.0)
        # The obstacle moves under an independent constant forward command.
        row: dict[str, Any] = {
            "world_step": observation.clock.step_index,
            "time": observation.clock.time,
            "robot": truth.__dict__,
            "estimated_robot": state.__dict__,
            "estimate_covariance": estimate.covariance,
            "estimated_gyro_bias": estimate.gyro_bias,
            "gyro_bias_std": estimate.gyro_bias_std,
            "true_gyro_bias": sensors.imu_biases["imu"],  # evaluation only
            "estimated_acceleration_bias": estimate.acceleration_bias,
            "acceleration_bias_std": estimate.acceleration_bias_std,
            "true_acceleration_bias": sensors.imu_acceleration_biases["imu"],
            "estimate_age_s": estimate_age,
            "position_error": math.hypot(state.x - truth.x, state.y - truth.y),
            "heading_error": abs(_angle_error(state.theta, truth.theta)),
            "safe_stop": safe_stop,
            "command": command.__dict__,
            "ready_packets": len(batch.ready),
            "dropped": list(batch.dropped),
            "pending_packets": batch.pending_count,
            "path_length": len(pipeline.path),
            "replanned": bool(latest and latest.replanned),
            "detections": len(latest.detections) if latest else 0,
            "tracks": [track.as_dict() for track in latest.tracks] if latest else [],
            "scan_matches": match_records,
            "goal_reached": goal_reached,
        }
        trace.append(row)
        if row["goal_reached"]:
            break
        world.step({
            "robot": command,
            "moving_obstacle": robotnav.ControlCommand(args.obstacle_speed, 0.0),
        })

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    result = {
        "schema_version": 1,
        "map": str(map_path),
        "seed": args.seed,
        "steps": len(trace),
        "lidar_packets": lidar_packets,
        "dropped_packets": total_dropped,
        "replans": replans,
        "goal_reached": bool(trace and trace[-1]["goal_reached"]),
        "estimator": "planar_ekf_" + args.localization,
        "configuration": {key: value for key, value in vars(args).items()
                          if key != "output"},
        "ekf_config": asdict(estimator.config),
        "accepted_motion_packets": estimator.accepted_motion_packets,
        "rejected_motion_packets": estimator.rejected_motion_packets,
        "rejected_estimator_packets": estimator.rejected_packets,
        "scan_match_attempts": match_attempts,
        "accepted_pose_corrections": estimator.accepted_pose_packets,
        "rejected_scan_matches": match_attempts - estimator.accepted_pose_packets,
        "stale_scans": stale_scans,
        "skipped_scans": skipped_scans,
        "safe_stop_steps": safe_stop_steps,
        "position_rmse": math.sqrt(sum(row["position_error"] ** 2
                                        for row in trace) / len(trace)),
        "trace": trace,
    }
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
                      encoding="utf-8")
    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--output", default="build/sensor-navigation/trace.json")
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--start", nargs=2, type=int, default=(2, 2))
    parser.add_argument("--goal", nargs=2, type=int, default=(12, 12))
    parser.add_argument("--obstacle", nargs=2, type=int, default=(9, 5))
    parser.add_argument("--obstacle-heading", type=float, default=math.pi / 2.0)
    parser.add_argument("--obstacle-speed", type=float, default=0.15)
    parser.add_argument("--obstacle-radius", type=float, default=0.45)
    parser.add_argument("--beams", type=int, default=91)
    parser.add_argument("--sensor-range", type=float, default=15.0)
    parser.add_argument("--ray-step", type=float, default=0.1)
    parser.add_argument("--sensor-noise", type=float, default=0.0)
    parser.add_argument("--odom-velocity-noise", type=float, default=0.02)
    parser.add_argument("--odom-heading-noise", type=float, default=0.002)
    parser.add_argument("--imu-acceleration-noise", type=float, default=0.05)
    parser.add_argument("--imu-yaw-rate-noise", type=float, default=0.01)
    parser.add_argument("--imu-gyro-bias", type=float, default=0.0,
                        help="constant gyro bias in rad/s")
    parser.add_argument("--imu-gyro-bias-random-walk", type=float, default=0.0,
                        help="gyro bias random walk in rad/s/sqrt(s)")
    parser.add_argument("--estimate-gyro-bias", action=argparse.BooleanOptionalAction,
                        default=True, help="enable the six-state EKF; disable for comparison")
    parser.add_argument("--ekf-gyro-bias-process-std", type=float, default=0.001,
                        help="EKF bias process std in rad/s/sqrt(s)")
    parser.add_argument("--imu-acceleration-bias", type=float, default=0.0,
                        help="constant forward acceleration bias in map units/s^2")
    parser.add_argument("--imu-acceleration-bias-random-walk", type=float, default=0.0,
                        help="forward acceleration bias random walk per sqrt(s)")
    parser.add_argument("--estimate-acceleration-bias", action=argparse.BooleanOptionalAction,
                        default=False, help="add forward acceleration bias to the EKF state")
    parser.add_argument("--ekf-acceleration-bias-process-std", type=float, default=0.001)
    parser.add_argument("--localization", choices=("dead_reckoning", "scan_matching"),
                        default="scan_matching")
    parser.add_argument("--initial-estimate-offset", nargs=3, type=float,
                        default=(0.0, 0.0, 0.0), metavar=("DX", "DY", "DTHETA"))
    parser.add_argument("--scan-match-every", type=int, default=5)
    parser.add_argument("--scan-match-translation-window", type=float, default=0.75)
    parser.add_argument("--scan-match-rotation-window", type=float, default=0.2)
    parser.add_argument("--scan-match-position-std", type=float, default=0.15)
    parser.add_argument("--scan-match-heading-std", type=float, default=0.03)
    parser.add_argument("--max-estimate-age", type=float, default=0.5)
    parser.add_argument("--max-scan-age", type=float, default=0.5)
    parser.add_argument("--latency-steps", type=int, default=1)
    parser.add_argument("--drop-probability", type=float, default=0.05)
    parser.add_argument("--timestamp-jitter-ns", type=int, default=0)
    parser.add_argument("--max-velocity", type=float, default=1.0)
    parser.add_argument("--max-steering", type=float, default=0.7)
    parser.add_argument("--goal-tolerance", type=float, default=0.75)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.scan_match_every <= 0:
        parser.error("--scan-match-every must be positive")
    for name in ("max_estimate_age", "max_scan_age", "scan_match_position_std",
                 "scan_match_heading_std", "scan_match_translation_window",
                 "scan_match_rotation_window"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and positive")
    for name in ("imu_gyro_bias_random_walk", "ekf_gyro_bias_process_std",
                 "imu_acceleration_bias_random_walk", "ekf_acceleration_bias_process_std"):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) < 0:
            parser.error(f"--{name.replace('_', '-')} must be finite and non-negative")
    for name in ("imu_gyro_bias", "imu_acceleration_bias"):
        if not math.isfinite(getattr(args, name)):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    result = run(args)
    print(json.dumps({key: result[key] for key in (
        "steps", "lidar_packets", "dropped_packets", "replans",
        "goal_reached")}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
