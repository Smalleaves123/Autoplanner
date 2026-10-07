#!/usr/bin/env python3
"""Generate reusable, connected multi-agent scenario manifests."""

from __future__ import annotations

import argparse
from collections import deque
import json
import random
from pathlib import Path


MANIFEST_TYPE = "robotnav.multi_agent.scenario_manifest"


def load_map(path: str | Path) -> list[str]:
    map_path = Path(path).expanduser()
    rows = [line.strip() for line in map_path.read_text(encoding="utf-8").splitlines()]
    rows = [row for row in rows if row]
    if not rows or any(set(row) - {"0", "1"} for row in rows):
        raise ValueError(f"invalid occupancy map: {map_path}")
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        raise ValueError(f"map rows have inconsistent widths: {map_path}")
    return rows


def _neighbors(point: tuple[int, int], width: int, height: int):
    x, y = point
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height:
            yield nx, ny


def largest_free_component(rows: list[str]) -> list[tuple[int, int]]:
    height, width = len(rows), len(rows[0])
    free = {
        (x, y)
        for y, row in enumerate(rows)
        for x, cell in enumerate(row)
        if cell == "0"
    }
    largest: set[tuple[int, int]] = set()
    while free:
        root = free.pop()
        component = {root}
        queue = deque([root])
        while queue:
            point = queue.popleft()
            for neighbor in _neighbors(point, width, height):
                if neighbor in free:
                    free.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
        if len(component) > len(largest):
            largest = component
    return sorted(largest, key=lambda point: (point[1], point[0]))


def shortest_path_distance(rows: list[str], start: tuple[int, int], goal: tuple[int, int]) -> int | None:
    if start == goal:
        return 0
    height, width = len(rows), len(rows[0])
    queue = deque([(start, 0)])
    visited = {start}
    while queue:
        point, distance = queue.popleft()
        for neighbor in _neighbors(point, width, height):
            if rows[neighbor[1]][neighbor[0]] != "0" or neighbor in visited:
                continue
            if neighbor == goal:
                return distance + 1
            visited.add(neighbor)
            queue.append((neighbor, distance + 1))
    return None


def generate_scenarios(
    map_path: str | Path,
    count: int,
    agents: int,
    seed: int,
    min_distance: int = 0,
) -> list[dict[str, object]]:
    if count <= 0 or agents <= 0 or min_distance < 0:
        raise ValueError("count and agents must be positive; min_distance must be non-negative")
    rows = load_map(map_path)
    cells = largest_free_component(rows)
    required = agents * 2
    if len(cells) < required:
        raise ValueError(
            f"largest free component has {len(cells)} cells but {required} are required")
    generator = random.Random(seed)
    scenarios: list[dict[str, object]] = []
    for index in range(count):
        scenario_seed = generator.randrange(0, 2**32)
        scenario_generator = random.Random(scenario_seed)
        selected = None
        distances: list[int] = []
        for _ in range(200):
            candidate = scenario_generator.sample(cells, required)
            candidate_distances = [
                shortest_path_distance(rows, candidate[agent], candidate[agents + agent])
                for agent in range(agents)
            ]
            if all(distance is not None and distance >= min_distance
                   for distance in candidate_distances):
                selected = candidate
                distances = [int(distance) for distance in candidate_distances]
                break
        if selected is None:
            raise ValueError(
                f"could not generate scenario {index} with min_distance={min_distance}")
        starts = selected[:agents]
        goals = selected[agents:]
        scenarios.append({
            "scenario_id": f"random_{index:03d}",
            "seed": scenario_seed,
            "starts": [list(point) for point in starts],
            "goals": [list(point) for point in goals],
            "difficulty": {
                "agent_count": agents,
                "min_shortest_path": min(distances),
                "max_shortest_path": max(distances),
                "mean_shortest_path": sum(distances) / len(distances),
                "total_shortest_path": sum(distances),
                "component_size": len(cells),
            },
        })
    return scenarios


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--scenarios", type=int, default=20)
    parser.add_argument("--agents", type=int, default=3)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--min-distance", type=int, default=10)
    args = parser.parse_args(argv)
    scenarios = generate_scenarios(
        args.map, args.scenarios, args.agents, args.seed, args.min_distance)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps({
        "schema_version": 1,
        "artifact_type": MANIFEST_TYPE,
        "map": str(Path(args.map).expanduser()),
        "seed": args.seed,
        "parameters": {
            "scenario_count": args.scenarios,
            "agents": args.agents,
            "min_distance": args.min_distance,
        },
        "scenarios": scenarios,
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
