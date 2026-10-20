#!/usr/bin/env python3
"""Multi-scan confirmation and stationary epoch handoff with late packets.

Global search is real on each generated scan. Relative motion comes from a
separate odometry-only EKF, never the simulated map pose. Moving recovery is
confirmed diagnostically but deliberately not applied by the stationary adapter.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import robotnav
from benchmark_relocalization import scenarios


def run(*, seed: int = 42) -> dict:
    definitions = scenarios()
    cases = []
    for name, rows, initial, wrong in (
        ("stationary_recovery", *definitions[0][1:]),
        ("moving_confirmation", *definitions[0][1:]),
        ("persistent_ambiguity", *definitions[1][1:]),
    ):
        moving = name == "moving_confirmation"
        suite = robotnav.SensorSuite(rows, lidars=[robotnav.Lidar2DConfig(
            num_beams=61, field_of_view=2 * math.pi, max_range=25,
            ray_step=0.05, range_noise_std=0.01)], seed=seed)
        relocalizer = robotnav.GridRelocalizer(rows)
        tracker = robotnav.RelocalizationTracker()
        speed = 0.2 if moving else 0.0
        motion = robotnav.PlanarEkf(robotnav.RobotState(0, 0, velocity=speed))
        epoch = robotnav.RelocalizationEpoch(wrong)
        trace = []
        transition = None
        old_pending = []
        queue_rejections = 0
        for index in range(4):
            capture_epoch = epoch.epoch_id
            stamp = (index + 1) * 500_000_000
            truth = replace(initial, x=initial.x + index * speed * 0.5 * math.cos(initial.theta),
                            y=initial.y + index * speed * 0.5 * math.sin(initial.theta), velocity=speed)
            observation = robotnav.WorldObservation(robotnav.SimulationClock(index, stamp * 1e-9, 0.5),
                (robotnav.EntityObservation("robot", truth),))
            odom = robotnav.OdometryMeasurement("odom", "robot", index, stamp, stamp,
                                               0, 0, 0, speed, 0.5 if index else 0)
            motion.update(odom)
            scan = suite.step(observation).ready[0]
            global_result = relocalizer.localize(scan)
            motion_capture = motion.estimate_at(stamp).state
            decision = tracker.update(global_result, timestamp_ns=stamp,
                                      motion_pose=motion_capture)
            if capture_epoch == 0:
                old_pending.append((capture_epoch, replace(odom, arrival_timestamp_ns=stamp + 1_000_000_000)))
            if decision.status == "confirmed" and not moving and epoch.epoch_id == 0:
                transition = epoch.apply(decision, now_ns=stamp, epoch_id=capture_epoch, stationary=True)
                # The external consumer clears old queued data by capture-time
                # epoch tag. Delivery is deliberately simulated after reset.
                for tag, pending in old_pending:
                    queue_rejections += int(not epoch.update_motion(pending, epoch_id=tag))
                old_pending.clear()
                # Motion-frame/tracker history is explicitly reset on switch.
                motion.reset(robotnav.RobotState(0, 0), timestamp_ns=stamp)
                tracker.reset()
            # Fresh post-boundary motion may update the new EKF, but recovery
            # alone does not mark map health as ready for control.
            if epoch.epoch_id and stamp > epoch.start_timestamp_ns:
                epoch.update_motion(odom, epoch_id=epoch.epoch_id)
            trace.append({"step": index, "timestamp_ns": stamp, "truth": asdict(truth),
                          "motion_pose": asdict(motion_capture),
                          "capture_epoch": capture_epoch,
                          "scan": scan.to_dict(), "global_result": asdict(global_result),
                          "decision": asdict(decision), "epoch_id": epoch.epoch_id,
                          "estimated_pose": asdict(epoch.ekf.estimate_at(stamp).state)})
        final = epoch.ekf.estimate.state
        confirmation = next((r for r in trace if r["decision"]["status"] == "confirmed"), None)
        cases.append({"case": name, "transition": asdict(transition) if transition else None,
                      "first_confirmation_step": confirmation["step"] if confirmation else None,
                      "confirmation_position_error": (math.hypot(
                          confirmation["decision"]["pose"]["x"] - confirmation["truth"]["x"],
                          confirmation["decision"]["pose"]["y"] - confirmation["truth"]["y"])
                          if confirmation else None),
                      "rejected_queued_packets": queue_rejections,
                      "final_position_error": math.hypot(final.x - truth.x, final.y - truth.y),
                      "epoch_id": epoch.epoch_id, "trace": trace,
                      "tracker_config": asdict(tracker.config),
                      "relocalizer_config": asdict(relocalizer.config)})
    return {"schema_version": 1, "seed": seed, "cases": cases,
            "limitations": ["ambiguous environments remain unresolved across identical scans",
                            "epoch adapter requires caller-established stationarity",
                            "automatic navigation and external component reset remain caller-owned"]}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default="build/sensor-navigation/relocalization-tracking-benchmark.json")
    args = parser.parse_args(argv)
    data = run(seed=args.seed)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(data, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps([{key: case[key] for key in ("case", "first_confirmation_step",
        "epoch_id", "rejected_queued_packets", "confirmation_position_error", "final_position_error")}
                     for case in data["cases"]], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
