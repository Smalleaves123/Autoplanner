#!/usr/bin/env python3
"""Run deterministic multi-agent Cooperative A* benchmark scenarios."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import robotnav
from multi_agent_scenarios import generate_scenarios


SCENARIOS = {
    "crossing": ([(2, 2), (8, 2)], [(8, 2), (2, 2)]),
    "four_way": (
        [(2, 2), (8, 2), (2, 8), (8, 8)],
        [(8, 8), (2, 8), (8, 2), (2, 2)],
    ),
    "converging": (
        [(2, 2), (8, 2), (2, 8)],
        [(5, 5), (5, 5), (5, 5)],
    ),
}


def _random_scenarios(
    map_path: Path,
    count: int,
    agents: int,
    seed: int,
) -> list[tuple[str, tuple[list[tuple[int, int]], list[tuple[int, int]]], int]]:
    """Generate deterministic assignments from one connected map component."""
    scenarios = generate_scenarios(map_path, count, agents, seed)
    return [(
        str(item["scenario_id"]),
        (
            [tuple(point) for point in item["starts"]],
            [tuple(point) for point in item["goals"]],
        ),
        int(item["seed"]),
    ) for item in scenarios]


def _manifest_scenarios(path: Path) -> list[tuple[str, tuple[list[tuple[int, int]], list[tuple[int, int]]], int]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    scenarios = data.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("scenario manifest must contain a non-empty scenarios list")
    result = []
    for index, item in enumerate(scenarios):
        if not isinstance(item, dict):
            raise ValueError(f"scenario {index} must be an object")
        starts = item.get("starts")
        goals = item.get("goals")
        if not isinstance(starts, list) or not isinstance(goals, list) or len(starts) != len(goals) or not starts:
            raise ValueError(f"scenario {index} has invalid starts/goals")
        try:
            start_points = [tuple(int(value) for value in point) for point in starts]
            goal_points = [tuple(int(value) for value in point) for point in goals]
        except (TypeError, ValueError) as error:
            raise ValueError(f"scenario {index} has invalid coordinates") from error
        if any(len(point) != 2 for point in start_points + goal_points):
            raise ValueError(f"scenario {index} points must have two coordinates")
        result.append((
            str(item.get("scenario_id", f"manifest_{index:03d}")),
            (start_points, goal_points),
            int(item.get("seed", data.get("seed", 0))),
        ))
    return result


def _write_trace(
    path: Path,
    starts: list[tuple[int, int]],
    goals: list[tuple[int, int]],
    result: Any,
) -> int:
    """Write a padded, long-form trace suitable for plotting and replay."""
    makespan = max((len(plan.path) for plan in result.plans), default=0)
    fields = (
        "time_step", "agent", "x", "y", "start_x", "start_y",
        "goal_x", "goal_y", "active", "wait", "planned", "success",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for agent, (start, goal) in enumerate(zip(starts, goals)):
            plan = result.plans[agent] if agent < len(result.plans) else None
            points = list(plan.path) if plan is not None else []
            planned = bool(plan is not None and plan.success)
            for time_step in range(makespan):
                if time_step < len(points):
                    point = points[time_step]
                elif points:
                    point = points[-1]
                else:
                    point = (float(start[0]), float(start[1]))
                previous = points[time_step - 1] if time_step > 0 and time_step < len(points) else point
                writer.writerow({
                    "time_step": time_step,
                    "agent": agent,
                    "x": point[0],
                    "y": point[1],
                    "start_x": start[0],
                    "start_y": start[1],
                    "goal_x": goal[0],
                    "goal_y": goal[1],
                    "active": planned and time_step < len(points),
                    "wait": planned and time_step > 0 and point == previous,
                    "planned": planned,
                    "success": planned,
                })
    return makespan


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--output-dir", default="autoplanner/results/multi_agent")
    parser.add_argument("--scenario-manifest", help="reuse a generated scenario manifest")
    parser.add_argument("--scenario", choices=("all", *SCENARIOS), default="all")
    parser.add_argument("--max-time-steps", type=int, default=120)
    parser.add_argument("--max-cbs-nodes", type=int, default=10000)
    parser.add_argument(
        "--algorithm", choices=("prioritized", "cbs", "both"), default="both",
        help="multi-agent solver or both for a side-by-side comparison")
    parser.add_argument(
        "--priority-order", default="",
        help="comma-separated input-agent order for prioritized planning")
    parser.add_argument(
        "--allow-shared-goals", action="store_true",
        help="allow agents to finish simultaneously at an identical goal")
    parser.add_argument(
        "--random-scenarios", type=int, default=0,
        help="append this many seeded random scenarios")
    parser.add_argument(
        "--random-agents", type=int, default=3,
        help="agents per random scenario (default: 3)")
    parser.add_argument(
        "--seed", type=int, default=20260927,
        help="seed for deterministic random scenario generation")
    args = parser.parse_args(argv)
    if args.max_time_steps <= 0:
        raise SystemExit("--max-time-steps must be positive")
    if args.max_cbs_nodes <= 0:
        raise SystemExit("--max-cbs-nodes must be positive")
    if args.random_scenarios < 0:
        raise SystemExit("--random-scenarios must be non-negative")
    if args.random_agents <= 0:
        raise SystemExit("--random-agents must be positive")
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    names = tuple(SCENARIOS) if args.scenario == "all" else (args.scenario,)
    algorithms = ("prioritized", "cbs") if args.algorithm == "both" else (args.algorithm,)
    if args.scenario_manifest and args.random_scenarios:
        raise SystemExit("--scenario-manifest cannot be combined with --random-scenarios")
    if args.scenario_manifest:
        try:
            scenario_specs = _manifest_scenarios(Path(args.scenario_manifest).expanduser())
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as error:
            raise SystemExit(f"invalid scenario manifest: {error}") from error
    else:
        scenario_specs = [
            (name, SCENARIOS[name], None) for name in names
        ]
    if args.random_scenarios:
        try:
            scenario_specs.extend(_random_scenarios(
                Path(args.map).expanduser(), args.random_scenarios,
                args.random_agents, args.seed))
        except ValueError as error:
            raise SystemExit(str(error)) from error
    rows = []
    for name, (starts, goals), scenario_seed in scenario_specs:
        if args.priority_order:
            try:
                priority_order = [int(value) for value in args.priority_order.split(",")]
            except ValueError as error:
                raise SystemExit("--priority-order must be comma-separated integers") from error
        else:
            priority_order = None
        for algorithm in algorithms:
            result = robotnav.plan_cooperative(
                args.map, starts, goals, allow_diagonal=False,
                allow_wait=True, max_time_steps=args.max_time_steps,
                algorithm=algorithm,
                allow_shared_goals=args.allow_shared_goals,
                priority_order=priority_order,
                max_cbs_nodes=args.max_cbs_nodes)
            trace_path = output_dir / f"trace_{algorithm}_{name}.csv"
            makespan = _write_trace(trace_path, starts, goals, result)
            wait_steps = sum(
                sum(1 for first, second in zip(plan.path, plan.path[1:])
                    if first == second)
                for plan in result.plans)
            total_length = sum(plan.path_length for plan in result.plans)
            rows.append({
                "scenario": name,
                "algorithm": algorithm,
                "allow_shared_goals": args.allow_shared_goals,
                "priority_order": priority_order or [],
                "max_cbs_nodes": args.max_cbs_nodes,
                "seed": scenario_seed if scenario_seed is not None else args.seed,
                "starts": starts,
                "goals": goals,
                "agents": len(starts),
                "success": result.success,
                "planned_agents": result.planned_agents,
                "vertex_conflicts": result.vertex_conflicts,
                "edge_conflicts": result.edge_conflicts,
                "makespan_steps": makespan,
                "wait_steps": wait_steps,
                "total_path_length": total_length,
                "trace_csv": trace_path.name,
                "agent_paths": [
                    [[x, y] for x, y in plan.path] for plan in result.plans
                ],
                "message": result.message,
            })
    with (output_dir / "multi_agent_benchmark.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    with (output_dir / "multi_agent_benchmark.json").open(
            "w", encoding="utf-8") as stream:
        by_algorithm = {}
        for algorithm in algorithms:
            algorithm_rows = [row for row in rows if row["algorithm"] == algorithm]
            successful_rows = [row for row in algorithm_rows if row["success"]]
            by_algorithm[algorithm] = {
                "runs": len(algorithm_rows),
                "successes": sum(bool(row["success"]) for row in algorithm_rows),
                "success_rate": (
                    sum(bool(row["success"]) for row in algorithm_rows)
                    / len(algorithm_rows) if algorithm_rows else 0.0),
                "successful_mean_makespan_steps": (
                    sum(row["makespan_steps"] for row in successful_rows)
                    / len(successful_rows) if successful_rows else 0.0),
                "successful_mean_total_path_length": (
                    sum(row["total_path_length"] for row in successful_rows)
                    / len(successful_rows) if successful_rows else 0.0),
                "mean_makespan_steps": (
                    sum(row["makespan_steps"] for row in algorithm_rows)
                    / len(algorithm_rows) if algorithm_rows else 0.0),
                "mean_total_path_length": (
                    sum(row["total_path_length"] for row in algorithm_rows)
                    / len(algorithm_rows) if algorithm_rows else 0.0),
            }
        json.dump({
            "benchmark": "multi_agent_cooperative_astar",
            "seed": args.seed,
            "summary": by_algorithm,
            "runs": rows,
        },
                  stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(output_dir / "multi_agent_benchmark.json")
    for row in rows:
        print(row["algorithm"], row["scenario"], row["success"], row["makespan_steps"],
              row["wait_steps"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
