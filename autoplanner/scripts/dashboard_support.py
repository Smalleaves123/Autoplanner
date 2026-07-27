"""Scenario persistence and prediction policy helpers for RobotNav Lab."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import hypot
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class MovingObstaclePrediction:
    """A constant-velocity obstacle forecast expressed in grid cells."""

    start_frame: int
    end_frame: int
    x: int
    y: int
    dx: int = 0
    dy: int = 0

    def positions(self) -> list[tuple[int, int]]:
        if self.end_frame < self.start_frame:
            raise ValueError("prediction end frame must not precede its start frame")
        return [
            (self.x + (frame - self.start_frame) * self.dx,
             self.y + (frame - self.start_frame) * self.dy)
            for frame in range(self.start_frame, self.end_frame + 1)
        ]

    def cli_values(self) -> tuple[int, int, int, int, int, int]:
        return (self.start_frame, self.end_frame, self.x, self.y,
                self.dx, self.dy)


@dataclass(frozen=True)
class RiskDecision:
    """A conservative action selected before dynamic-pipeline execution."""

    action: str
    velocity_scale: float
    minimum_clearance: float
    reason: str


def point_to_segment_distance(point: tuple[int, int],
                              start: tuple[int, int],
                              goal: tuple[int, int]) -> float:
    """Return the Euclidean distance from a grid point to a line segment."""
    px, py = point
    sx, sy = start
    gx, gy = goal
    dx = gx - sx
    dy = gy - sy
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return hypot(px - sx, py - sy)
    projection = ((px - sx) * dx + (py - sy) * dy) / length_squared
    projection = max(0.0, min(1.0, projection))
    nearest_x = sx + projection * dx
    nearest_y = sy + projection * dy
    return hypot(px - nearest_x, py - nearest_y)


def decide_prediction_risk(prediction: MovingObstaclePrediction,
                           start: tuple[int, int], goal: tuple[int, int],
                           robot_radius: float) -> RiskDecision:
    """Choose monitor, slower replanning, or a pre-flight safe stop.

    The policy deliberately uses the direct start-goal corridor as a cheap,
    planner-independent pre-flight screen. The dynamic C++ pipeline remains
    responsible for path-specific collision checks and D* Lite replanning.
    """
    positions = prediction.positions()
    minimum_clearance = min(
        point_to_segment_distance(position, start, goal)
        for position in positions
    )
    start_clearance = min(hypot(x - start[0], y - start[1])
                          for x, y in positions)
    hard_clearance = max(robot_radius + 0.5, 1.0)
    if start_clearance <= hard_clearance:
        return RiskDecision(
            action="safe_stop",
            velocity_scale=0.0,
            minimum_clearance=minimum_clearance,
            reason="predicted obstacle enters the robot start safety envelope",
        )
    if minimum_clearance <= hard_clearance:
        return RiskDecision(
            action="replan_slow",
            velocity_scale=0.55,
            minimum_clearance=minimum_clearance,
            reason="predicted obstacle intersects the nominal safety corridor",
        )
    if minimum_clearance <= 2.0 * hard_clearance:
        return RiskDecision(
            action="replan_slow",
            velocity_scale=0.75,
            minimum_clearance=minimum_clearance,
            reason="predicted obstacle is close to the nominal safety corridor",
        )
    return RiskDecision(
        action="monitor",
        velocity_scale=1.0,
        minimum_clearance=minimum_clearance,
        reason="predicted obstacle remains outside the pre-flight safety corridor",
    )


def load_grid(path: Path) -> list[list[int]]:
    rows = [line.strip() for line in path.read_text().splitlines() if line.strip()]
    if not rows or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError(f"map must contain non-empty rectangular rows: {path}")
    return [[1 if cell in "1#@" else 0 for cell in row] for row in rows]


def grid_lines(grid: list[list[int]]) -> list[str]:
    if not grid or any(len(row) != len(grid[0]) for row in grid):
        raise ValueError("grid must contain non-empty rectangular rows")
    return ["".join("1" if cell else "0" for cell in row) for row in grid]


def grid_from_lines(lines: list[str]) -> list[list[int]]:
    if not isinstance(lines, list):
        raise ValueError("scene grid must be a list of strings")
    rows = [str(row).strip() for row in lines if str(row).strip()]
    if not rows or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError("scene grid must contain non-empty rectangular rows")
    if any(cell not in "01" for row in rows for cell in row):
        raise ValueError("scene grid may contain only 0 and 1")
    return [[int(cell) for cell in row] for row in rows]


def write_grid(path: Path, grid: list[list[int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(grid_lines(grid)) + "\n")


def save_scene(path: Path, base_map: str, grid: list[list[int]],
               settings: dict[str, Any],
               prediction: MovingObstaclePrediction | None) -> None:
    """Persist a portable dashboard scene without local absolute paths."""
    path.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "schema_version": 1,
        "base_map": base_map,
        "grid": grid_lines(grid),
        "settings": settings,
        "prediction": asdict(prediction) if prediction else None,
    }
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))


def load_scene(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("unsupported dashboard scene")
    settings = data.get("settings")
    if not isinstance(settings, dict):
        raise ValueError("scene settings must be a mapping")
    prediction_data = data.get("prediction")
    prediction = (
        MovingObstaclePrediction(**prediction_data)
        if isinstance(prediction_data, dict) else None
    )
    return {
        "base_map": str(data.get("base_map", "")),
        "grid": grid_from_lines(data.get("grid", [])),
        "settings": settings,
        "prediction": prediction,
    }
