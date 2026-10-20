#!/usr/bin/env python3
"""Deterministic localization failure cases, including an undetectable alias.

Truth is used only for scan generation/error evaluation, never by the health
monitor, matcher or EKF. Repeated-room aliasing is explicitly reported as a
limitation, not expected to be solved by local health rules.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import robotnav


ROOM = ["1" * 20] + ["1" + "0" * 18 + "1" for _ in range(14)] + ["1" * 20]


def run(*, seed: int = 42) -> dict:
    results = []
    for case in ("healthy", "dynamic_only", "single_wall", "teleport",
                 "lidar_outage", "motion_outage", "repeated_room_alias"):
        rows = ROOM
        if case == "repeated_room_alias":
            rows = ["1" * 21] + ["1" + "0" * 9 + "1" + "0" * 9 + "1"
                                  for _ in range(13)] + ["1" * 21]
        elif case == "single_wall":
            rows = ["1" * 11] + ["1" + "0" * 9 + "1" for _ in range(13)] + ["1" * 11]
        initial = robotnav.RobotState(5.4, 6.3)
        sensors = robotnav.SensorSuite(rows, lidars=[robotnav.Lidar2DConfig(
            num_beams=61, field_of_view=2 * math.pi, max_range=9
            if case == "repeated_room_alias" else 25, ray_step=0.05)], seed=seed)
        matcher = robotnav.GridScanMatcher(rows, robotnav.ScanMatcherConfig(min_points=6))
        monitor = robotnav.LocalizationHealthMonitor(robotnav.LocalizationHealthConfig(
            max_motion_age=0.75, max_scan_age=0.75, max_match_age=1.5))
        estimator = robotnav.PlanarEkf(initial)
        trace = []
        transitions = []
        last_status = None
        last_scan = last_motion = None
        for index in range(10):
            stamp = index * 500_000_000
            # Two healthy samples, four anomalous samples, then recovery.
            # Alias persists: its recovery cannot be inferred from local fit.
            anomaly = case != "healthy" and index >= 2 and (
                index < 6 or case == "repeated_room_alias")
            truth = initial
            if anomaly and case == "teleport":
                truth = replace(initial, x=initial.x + 4, y=initial.y + 3)
            if anomaly and case == "repeated_room_alias":
                truth = replace(initial, x=initial.x + 10)
            observation = robotnav.WorldObservation(
                robotnav.SimulationClock(index, index * 0.5, 0.5),
                (robotnav.EntityObservation("robot", truth),))
            scan = sensors.step(observation).ready[0]
            if not (anomaly and case == "motion_outage"):
                # Stationary zero wheel-rate input. Teleport is not reported as
                # wheel displacement; no truth pose/delta enters the estimator.
                estimator.update(robotnav.OdometryMeasurement(
                    "odom", "robot", index, stamp, stamp, 0, 0, 0, 0,
                    0.5 if index else 0))
                estimator.update(robotnav.ImuMeasurement(
                    "imu", "robot", index, stamp, stamp, 0, 0))
                last_motion = stamp
            match_record = None
            if not (anomaly and case == "lidar_outage"):
                if anomaly and case == "dynamic_only":
                    scan = replace(scan, kinds=("dynamic",) * len(scan.ranges))
                if anomaly and case == "single_wall":
                    ys = [2 + 0.5 * k for k in range(22)]
                    scan = replace(scan,
                        angles=tuple(math.atan2(y - truth.y, 10 - truth.x) for y in ys),
                        ranges=tuple(math.hypot(y - truth.y, 10 - truth.x) for y in ys),
                        hits=(True,) * len(ys), kinds=("static",) * len(ys))
                hint = estimator.estimate_at(stamp).state
                match = matcher.match(scan, hint)
                assessment = monitor.assess_match(match, hint)
                fused = False
                if assessment.accepted:
                    fused = estimator.update(robotnav.PoseMeasurement(
                        "map", "robot", index, stamp, stamp,
                        match.pose.x, match.pose.y, match.pose.theta))
                monitor.record_match(stamp, accepted=fused,
                    reason=assessment.reason if not assessment.accepted else
                    "matched" if fused else "ekf_rejected")
                match_record = {"match": asdict(match), "assessment": asdict(assessment),
                                "fused": fused}
                last_scan = stamp
            estimate = estimator.estimate_at(stamp)
            health = monitor.evaluate(stamp, estimate, motion_timestamp_ns=last_motion,
                                      scan_timestamp_ns=last_scan)
            error = math.hypot(estimate.state.x - truth.x, estimate.state.y - truth.y)
            row = {"step": index, "time": index * 0.5, "anomaly": anomaly,
                   "truth": asdict(truth), "estimate": asdict(estimate.state),
                   "covariance": estimate.covariance,
                   "position_error": error, "health": asdict(health), "scan_match": match_record,
                   "scan": (scan.to_dict() if last_scan == stamp else None)}
            trace.append(row)
            if health.status != last_status:
                transitions.append({"step": index, "from": last_status, "to": health.status,
                                    "reasons": health.reasons})
                last_status = health.status
        abnormal = [row for row in trace if row["anomaly"]]
        stop_steps = [row["step"] for row in abnormal if row["health"]["safe_stop"]]
        wrong_tracking = [row["step"] for row in trace if row["position_error"] > 1 and
                          row["health"]["status"] == "tracking"]
        recovered = [row["step"] for row in trace if row["step"] >= 6 and
                     not row["anomaly"] and row["health"]["status"] == "tracking"]
        results.append({"case": case,
                        "expectation": "limitation_wrong_tracking" if case == "repeated_room_alias"
                                       else "healthy" if case == "healthy" else "detect_and_recover",
                        "first_anomaly_stop_step": stop_steps[0] if stop_steps else None,
                        "detection_delay_s": (0.5 * (stop_steps[0] - 2) if stop_steps else None),
                        "first_recovery_step": recovered[0] if recovered and abnormal else None,
                        "wrong_tracking_steps": wrong_tracking,
                        "final_status": trace[-1]["health"]["status"],
                        "max_position_error": max(row["position_error"] for row in trace),
                        "map": rows, "matcher_config": asdict(matcher.config),
                        "health_config": asdict(monitor.config),
                        "transitions": transitions, "trace": trace})
    return {"schema_version": 1, "seed": seed,
            "limitations": ["local matching cannot disambiguate identical rooms",
                            "EKF covariance and health are not correctness guarantees"],
            "cases": results}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default="build/sensor-navigation/localization-health-benchmark.json")
    args = parser.parse_args(argv)
    result = run(seed=args.seed)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
                      encoding="utf-8")
    print(json.dumps([{key: case[key] for key in (
        "case", "first_anomaly_stop_step", "wrong_tracking_steps", "final_status")}
                     for case in result["cases"]], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
