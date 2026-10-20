#!/usr/bin/env python3
"""Known-map global recovery versus a far-away local pose hint.

The teleport remains in place; unlike the health fault benchmark, recovery
cannot come from restoring truth to the original position. Symmetric maps
must return ambiguity rather than resetting the estimator to an arbitrary mode.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import robotnav


def scenarios() -> list[tuple[str, list[str], robotnav.RobotState, robotnav.RobotState]]:
    unique = ["1" * 16] + ["1" + "0" * 14 + "1" for _ in range(10)] + ["1" * 16]
    unique[3] = "1001110000000001"
    unique[4] = "1001110000000001"
    unique[8] = "1000000000011001"
    unique[9] = "1000000000011001"
    repeated = ["1" * 21] + ["1" + "0" * 9 + "1" + "0" * 9 + "1"
                             for _ in range(13)] + ["1" * 21]
    symmetric = ["1" * 12] + ["1" + "0" * 10 + "1" for _ in range(10)] + ["1" * 12]
    return [("persistent_teleport", unique, robotnav.RobotState(9.4, 6.3, 0.12),
             robotnav.RobotState(2.5, 7.5, -0.7)),
            ("repeated_rooms", repeated, robotnav.RobotState(15.4, 6.3, 0.12),
             robotnav.RobotState(5.4, 6.3, 0.12)),
            ("symmetric_room", symmetric, robotnav.RobotState(4.4, 6.3, 0.12),
             robotnav.RobotState(2.5, 2.5)),
            ("dynamic_only", unique, robotnav.RobotState(9.4, 6.3, 0.12),
             robotnav.RobotState(2.5, 7.5))]


def run(*, seed: int = 42) -> dict:
    cases = []
    for name, rows, truth, wrong_hint in scenarios():
        observation = robotnav.WorldObservation(robotnav.SimulationClock(1, 0.5, 0.5),
            (robotnav.EntityObservation("robot", truth),))
        suite = robotnav.SensorSuite(rows, lidars=[robotnav.Lidar2DConfig(
            num_beams=61, field_of_view=2 * math.pi, max_range=25, ray_step=0.05)], seed=seed)
        scan = suite.step(observation).ready[0]
        if name == "dynamic_only":
            scan = replace(scan, kinds=("dynamic",) * len(scan.ranges))
        local = robotnav.GridScanMatcher(rows).match(scan, wrong_hint)
        global_matcher = robotnav.GridRelocalizer(rows)
        result = global_matcher.localize(scan)
        estimator = robotnav.PlanarEkf(wrong_hint)
        before = estimator.estimate.state
        # Explicit epoch reset only for a non-ambiguous global candidate.
        # No old-frame packets are fed after this reset in this benchmark.
        if result.accepted:
            estimator.reset(result.pose, timestamp_ns=scan.timestamp_ns)
        final = estimator.estimate.state
        cases.append({"case": name, "map": rows, "truth": asdict(truth),
                      "initial_estimate": asdict(before), "final_estimate": asdict(final),
                      "initial_position_error": math.hypot(before.x - truth.x, before.y - truth.y),
                      "final_position_error": math.hypot(final.x - truth.x, final.y - truth.y),
                      "ekf_reset": result.accepted, "local_match": asdict(local),
                      "global_result": asdict(result), "scan": scan.to_dict(),
                      "configuration": asdict(global_matcher.config)})
    return {"schema_version": 1, "seed": seed, "cases": cases,
            "limitations": ["finite grid/top-seed search does not prove uniqueness",
                            "one scan does not provide temporal confirmation",
                            "global reset requires explicit packet/history epoch handling"]}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--output", default="build/sensor-navigation/relocalization-benchmark.json")
    args = parser.parse_args(argv)
    result = run(seed=args.seed)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
                      encoding="utf-8")
    print(json.dumps([{ "case": case["case"], "reason": case["global_result"]["reason"],
                        "hypotheses": len(case["global_result"]["hypotheses"]),
                        "position_error": case["final_position_error"]}
                      for case in result["cases"]], sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
