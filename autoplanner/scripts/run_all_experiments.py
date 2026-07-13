#!/usr/bin/env python3
"""Batch-run all planners across all maps and collect results.

This script drives the C++ CLI to generate benchmark data. It runs every
(planner, map) pair and writes a combined CSV summary.

Usage:
    python scripts/run_all_experiments.py --build_dir build --data_dir data --output_dir results/benchmark
"""
import argparse
import csv
import json
import os
import subprocess
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
AUTOPLANNER_DIR = SCRIPT_DIR.parent


PLANNERS = {
    "dijkstra":   [],
    "astar":      [],
    "weighted_astar": ["--weight", "1.5"],
    "improved_astar": ["--robot-radius", "1.0"],
    "jps":        [],
    "dstar_lite": [],
    "rrt":        [],
    "rrt_star":   [],
    "informed_rrt_star": [],
    "bi_rrt":     [],
    "hybrid_astar": [],
}

MAPS = [
    ("simple_50x50",            "maps/simple_50x50.txt",            (1, 1), (48, 48)),
    ("maze_100x100",            "maps/maze_100x100.txt",            (1, 1), (98, 98)),
    ("warehouse_100x100",       "maps/warehouse_100x100.txt",       (3, 5), (90, 80)),
    ("random_100x100_density_20", "maps/random_100x100_density_20.txt", (1, 1), (98, 98)),
]


def run_planner(cli: str, planner: str, extra_args: list[str],
                map_path: str, start: tuple, goal: tuple,
                output_dir: str) -> dict:
    """Run a single planner and return parsed metrics."""
    cmd = [
        cli,
        "--planner", planner,
        "--map", map_path,
        "--start", str(start[0]), str(start[1]),
        "--goal", str(goal[0]), str(goal[1]),
        "--output", output_dir,
    ] + extra_args

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    metrics_path = Path(output_dir) / "metrics.json"
    data = {
        "planner": planner,
        "map": os.path.basename(map_path),
        "success": False,
        "time_ms": 0.0,
        "path_length": 0.0,
        "expanded_nodes": 0,
    }

    if metrics_path.exists():
        try:
            with metrics_path.open() as f:
                metrics = json.load(f)
            data.update({
                "success": bool(metrics.get("success", False)),
                "time_ms": float(metrics.get("planning_time_ms", 0.0)),
                "path_length": float(metrics.get("path_length", 0.0)),
                "expanded_nodes": int(metrics.get("expanded_nodes", 0)),
            })
        except (OSError, ValueError, TypeError):
            pass

    if result.returncode != 0:
        data["success"] = False
    return data


def main():
    ap = argparse.ArgumentParser(description="Batch benchmark runner")
    ap.add_argument("--build_dir", default="build", help="CMake build directory")
    ap.add_argument("--data_dir", default=str(AUTOPLANNER_DIR / "data"),
                    help="Data directory root")
    ap.add_argument("--output_dir", default="results/benchmark",
                    help="Output directory for results")
    ap.add_argument("--planners", nargs="*",
                    default=list(PLANNERS.keys()),
                    help="Planners to run (default: all)")
    ap.add_argument("--repeat", type=int, default=1,
                    help="Number of repeats per (planner, map) pair")
    args = ap.parse_args()

    cli = os.path.abspath(os.path.join(args.build_dir, "apps", "autoplanner_cli"))
    if not os.path.exists(cli):
        print(f"CLI not found: {cli}. Build the project first.", file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    map_root = Path(args.data_dir)
    maps = [
        (name, str(map_root / relative_path), start, goal)
        for name, relative_path, start, goal in MAPS
    ]

    all_results = []
    total = len(args.planners) * len(maps) * args.repeat
    completed = 0

    for rep in range(args.repeat):
        for planner in args.planners:
            extra = PLANNERS.get(planner, [])
            for map_name, map_path, start, goal in maps:
                if not os.path.exists(map_path):
                    print(f"  [SKIP] map not found: {map_path}")
                    completed += 1
                    continue

                out_dir = os.path.join(args.output_dir, f"{planner}_{map_name}_{rep}")
                os.makedirs(out_dir, exist_ok=True)

                completed += 1
                print(f"[{completed}/{total}] {planner} on {map_name} ... ", end="", flush=True)

                try:
                    row = run_planner(cli, planner, extra, map_path,
                                      start, goal, out_dir)
                    all_results.append(row)
                    status = "OK" if row["success"] else "FAIL"
                    print(f"{status} ({row['time_ms']:.2f} ms)")
                except Exception as e:
                    print(f"ERROR: {e}")
                    all_results.append({
                        "planner": planner,
                        "map": os.path.basename(map_path),
                        "success": False,
                        "time_ms": 0.0,
                        "path_length": 0.0,
                        "expanded_nodes": 0,
                    })

    # Write summary CSV
    csv_path = os.path.join(args.output_dir, "all_results.csv")
    fieldnames = ["planner", "map", "success", "time_ms", "path_length", "expanded_nodes"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(all_results)

    print(f"\nDone. {len(all_results)} results written to {csv_path}")


if __name__ == "__main__":
    main()
