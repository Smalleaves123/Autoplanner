#!/usr/bin/env python3
"""Validate a long-form multi-agent trace against a grid map."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def _load_map(path: Path) -> list[str]:
    rows = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
    rows = [row for row in rows if row]
    if not rows or any(set(row) - {"0", "1"} for row in rows):
        raise ValueError(f"invalid occupancy map: {path}")
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        raise ValueError(f"map rows have inconsistent widths: {path}")
    return rows


def _load_trace(path: Path) -> dict[int, list[dict[str, str]]]:
    agents: dict[int, list[dict[str, str]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            agents[int(row["agent"])].append(row)
    for rows in agents.values():
        rows.sort(key=lambda row: int(row["time_step"]))
    return dict(agents)


def validate(
    map_rows: list[str],
    agents: dict[int, list[dict[str, str]]],
    allow_shared_goals: bool,
) -> dict[str, object]:
    positions: dict[int, dict[int, tuple[int, int]]] = defaultdict(dict)
    active: dict[int, dict[int, bool]] = defaultdict(dict)
    goals: dict[int, tuple[int, int]] = {}
    errors: list[str] = []
    height, width = len(map_rows), len(map_rows[0])
    for agent, rows in agents.items():
        for row in rows:
            time_step = int(row["time_step"])
            x, y = int(float(row["x"])), int(float(row["y"]))
            positions[time_step][agent] = (x, y)
            active[time_step][agent] = row.get("active", "true").lower() == "true"
            goals[agent] = (int(float(row["goal_x"])), int(float(row["goal_y"])))
            if not (0 <= x < width and 0 <= y < height):
                errors.append(f"agent {agent} leaves map at t={time_step}")
            elif map_rows[y][x] != "0":
                errors.append(f"agent {agent} enters obstacle at t={time_step}")

    vertex_conflicts = 0
    edge_conflicts = 0
    times = sorted(positions)
    for time_step in times:
        occupied: dict[tuple[int, int], int] = {}
        for agent, point in positions[time_step].items():
            if not active[time_step].get(agent, False):
                continue
            previous = occupied.get(point)
            if previous is not None:
                shared_goal = (
                    allow_shared_goals and goals.get(previous) == point and
                    goals.get(agent) == point)
                if not shared_goal:
                    vertex_conflicts += 1
            occupied[point] = agent
        if time_step == 0:
            continue
        previous_positions = positions.get(time_step - 1, {})
        for first, first_to in positions[time_step].items():
            if not active[time_step].get(first, False) or not active[time_step - 1].get(first, False):
                continue
            for second, second_to in positions[time_step].items():
                if second <= first or not active[time_step].get(second, False) or not active[time_step - 1].get(second, False):
                    continue
                first_from = previous_positions.get(first)
                second_from = previous_positions.get(second)
                if first_from == second_to and second_from == first_to and first_from != first_to:
                    edge_conflicts += 1

    return {
        "valid": not errors and vertex_conflicts == 0 and edge_conflicts == 0,
        "agents": len(agents),
        "time_steps": len(times),
        "vertex_conflicts": vertex_conflicts,
        "edge_conflicts": edge_conflicts,
        "errors": errors,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--trace", required=True)
    parser.add_argument("--output", help="optional JSON report path")
    parser.add_argument("--allow-shared-goals", action="store_true")
    args = parser.parse_args(argv)
    report = validate(
        _load_map(Path(args.map).expanduser()),
        _load_trace(Path(args.trace).expanduser()),
        args.allow_shared_goals,
    )
    if args.output:
        output = Path(args.output).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
