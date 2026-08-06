#!/usr/bin/env python3
"""Minimal Python example for the RobotNav planning facade."""

from pathlib import Path

import robotnav


ROOT = Path(__file__).resolve().parents[2]
result = robotnav.plan(
    ROOT / "autoplanner/data/maps/simple_50x50.txt",
    start=(1, 1),
    goal=(20, 20),
    config=robotnav.PlannerConfig(
        planner="improved_astar",
        robot_radius=0.0,
    ),
)

print(f"success={result.success} status={result.status_code}")
print(f"points={len(result.path)} length={result.path_length:.3f} m")
print(f"planning_time={result.planning_time_ms:.3f} ms")
