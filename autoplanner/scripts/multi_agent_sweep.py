#!/usr/bin/env python3
"""Sweep multi-agent solver settings over deterministic random scenarios."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import time
from pathlib import Path

import robotnav

from multi_agent_benchmark import _random_scenarios


def _ints(value: str, flag: str) -> list[int]:
    try:
        values = [int(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as error:
        raise SystemExit(f"{flag} must be comma-separated integers") from error
    if not values or any(item <= 0 for item in values):
        raise SystemExit(f"{flag} must contain positive integers")
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", default="autoplanner/data/maps/simple_50x50.txt")
    parser.add_argument("--output-dir", default="autoplanner/results/multi_agent_sweep")
    parser.add_argument("--seeds", default="20260927,20260928")
    parser.add_argument("--agent-counts", default="2,3,4")
    parser.add_argument("--cbs-node-budgets", default="1000,5000,10000")
    parser.add_argument("--scenarios-per-seed", type=int, default=5)
    parser.add_argument("--max-time-steps", type=int, default=120)
    args = parser.parse_args(argv)
    if args.scenarios_per_seed <= 0 or args.max_time_steps <= 0:
        raise SystemExit("scenario and time limits must be positive")

    seeds = _ints(args.seeds, "--seeds")
    agent_counts = _ints(args.agent_counts, "--agent-counts")
    node_budgets = _ints(args.cbs_node_budgets, "--cbs-node-budgets")
    map_path = Path(args.map).expanduser()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, object]] = []
    scenario_manifest: list[dict[str, object]] = []
    for agents in agent_counts:
        for seed in seeds:
            scenarios = _random_scenarios(
                map_path, args.scenarios_per_seed, agents, seed)
            scenario_manifest.append({
                "seed": seed,
                "agents": agents,
                "scenarios": [
                    {"name": name, "starts": starts, "goals": goals}
                    for name, (starts, goals), _ in scenarios
                ],
            })
            for max_cbs_nodes in node_budgets:
                for algorithm in ("prioritized", "cbs"):
                    outcomes = []
                    durations = []
                    for _, (starts, goals), _ in scenarios:
                        started = time.perf_counter()
                        result = robotnav.plan_cooperative(
                            map_path,
                            starts,
                            goals,
                            allow_diagonal=False,
                            allow_wait=True,
                            max_time_steps=args.max_time_steps,
                            algorithm=algorithm,
                            max_cbs_nodes=max_cbs_nodes,
                        )
                        durations.append((time.perf_counter() - started) * 1000.0)
                        outcomes.append(result)
                    successful = [result for result in outcomes if result.success]
                    rows.append({
                        "algorithm": algorithm,
                        "seed": seed,
                        "agents": agents,
                        "scenarios": len(outcomes),
                        "max_time_steps": args.max_time_steps,
                        "max_cbs_nodes": max_cbs_nodes,
                        "successes": len(successful),
                        "success_rate": len(successful) / len(outcomes),
                        "mean_runtime_ms": statistics.fmean(durations),
                        "p95_runtime_ms": sorted(durations)[
                            max(0, math.ceil(len(durations) * 0.95) - 1)],
                        "mean_successful_makespan": statistics.fmean(
                            max(len(plan.path) for plan in result.plans)
                            for result in successful) if successful else 0.0,
                        "mean_successful_path_length": statistics.fmean(
                            sum(plan.path_length for plan in result.plans)
                            for result in successful) if successful else 0.0,
                        "mean_vertex_conflicts": statistics.fmean(
                            result.vertex_conflicts for result in outcomes),
                        "mean_edge_conflicts": statistics.fmean(
                            result.edge_conflicts for result in outcomes),
                    })

    csv_path = output_dir / "multi_agent_sweep.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    json_path = output_dir / "multi_agent_sweep.json"
    with json_path.open("w", encoding="utf-8") as stream:
        json.dump({
            "benchmark": "multi_agent_solver_sweep",
            "map": str(map_path),
            "parameters": {
                "seeds": seeds,
                "agent_counts": agent_counts,
                "cbs_node_budgets": node_budgets,
                "scenarios_per_seed": args.scenarios_per_seed,
                "max_time_steps": args.max_time_steps,
            },
            "scenario_manifest": scenario_manifest,
            "runs": rows,
        }, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
