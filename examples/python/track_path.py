#!/usr/bin/env python3
"""Plan, generate a trajectory, and run Stanley in the Python facade."""

from pathlib import Path

import robotnav


ROOT = Path(__file__).resolve().parents[2]
planned = robotnav.plan(
    ROOT / "autoplanner/data/maps/simple_50x50.txt",
    (1, 1), (20, 20),
)
trajectory = robotnav.generate_trajectory(
    planned, robotnav.TrajectoryConfig(target_velocity=1.0))
controller = robotnav.Controller(
    robotnav.ControllerConfig(controller="stanley"))
simulation = robotnav.simulate(
    robotnav.RobotState(1.0, 1.0), trajectory, controller, max_time=5.0)

print(f"planned={planned.success} points={len(planned.path)}")
print(f"trajectory_points={len(trajectory.points)} length={trajectory.length:.3f}")
print(f"simulation_steps={len(simulation.states)}")
print(f"max_cross_track={simulation.metrics.max_cross_track:.3f}")
