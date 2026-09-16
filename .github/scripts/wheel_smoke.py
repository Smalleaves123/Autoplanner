#!/usr/bin/env python3
"""Smoke-test an installed RobotNav wheel outside the source checkout."""

from __future__ import annotations

from pathlib import Path
import sys

import autoplanner
import autompc
import robotnav
import robotnav._robotnav  # noqa: F401


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: wheel_smoke.py MAP")
    map_path = Path(sys.argv[1]).resolve()
    grid = autoplanner.GridMap()
    assert grid.load_from_txt(str(map_path))
    planned = autoplanner.plan(
        "astar", grid, autoplanner.Point2i(1, 1),
        autoplanner.Point2i(48, 48))
    assert planned.success, planned.message
    assert planned.path_length > 0.0

    options = autompc.SimulationOptions()
    simulator = autompc.KinematicBicycleSimulator(
        autompc.State(0.0, 0.0, 0.0, 0.0), options)
    state = simulator.step(autompc.Control(1.0, 0.1))
    assert state.x > 0.0

    differential = autompc.DifferentialDriveSimulator(
        autompc.State(0.0, 0.0, 0.0, 0.0),
        autompc.DifferentialDriveOptions())
    differential_state = differential.step(
        autompc.DifferentialDriveCommand(1.0, 0.5))
    assert differential_state.x > 0.0
    assert differential.right_wheel_velocity > differential.left_wheel_velocity

    facade_trajectory = robotnav.generate_trajectory(
        [(0.0, 0.0), (3.0, 0.0)])
    facade_controller = robotnav.Controller(
        robotnav.ControllerConfig(controller="stanley"))
    facade_result = robotnav.simulate(
        robotnav.RobotState(0.0, 0.0),
        facade_trajectory,
        facade_controller,
        robotnav.SimulationConfig(execution_model="differential_drive"),
        max_time=0.2,
    )
    assert facade_result.states
    assert facade_result.execution_model == "differential_drive"
    assert hasattr(autompc, "MPCController")
    assert robotnav.__version__ == "0.6.0"

    scenario = robotnav.DynamicScenario(
        map_path, (1, 1), (8, 8),
        robotnav.DynamicConfig(
            frames=1, steps_per_frame=2, auto_insert_obstacles=False))
    dynamic = robotnav.run_dynamic_scenario(scenario)
    assert dynamic.trace
    restored = robotnav.DynamicResult.from_dict(dynamic.to_dict())
    assert restored.status_code == dynamic.status_code
    print("installed RobotNav wheel smoke test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
