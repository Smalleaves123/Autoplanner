#!/usr/bin/env python3
"""Run a short dynamic-navigation scenario through the Python facade."""

from pathlib import Path

import robotnav


ROOT = Path(__file__).resolve().parents[2]
result = robotnav.run_dynamic(
    ROOT / "autoplanner/data/maps/simple_50x50.txt",
    (1, 1), (12, 12),
    robotnav.DynamicConfig(
        planner="space_time_astar",
        controller="stanley",
        local_planner="dwa",
        frames=12,
        steps_per_frame=30,
        auto_insert_obstacles=False,
        moving_obstacles=(robotnav.MovingObstacle(
            0, 7, 7, 4, 0, 1, radius=0.5),),
    ),
)

print(f"status={result.status_code} success={result.success}")
print(f"trace_steps={len(result.trace)} replans={result.metrics['replanning_count']}")
print(f"goal_reached={result.metrics['goal_reached']}")
