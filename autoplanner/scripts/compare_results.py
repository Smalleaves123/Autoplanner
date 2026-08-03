#!/usr/bin/env python3
"""Summarize the real CSV/JSON benchmark ledger."""
import argparse
import csv
import statistics
import sys
from collections import defaultdict
from pathlib import Path


def load_rows(path: Path) -> list[dict]:
    try:
        with path.open(newline="") as stream:
            return list(csv.DictReader(stream))
    except FileNotFoundError:
        print(f"File not found: {path}", file=sys.stderr)
        return []


def as_bool(value) -> bool:
    return str(value).lower() in ("1", "true", "yes")


def as_float(row: dict, key: str) -> float:
    try:
        return float(row.get(key, 0.0))
    except (TypeError, ValueError):
        return 0.0


def as_int(row: dict, key: str) -> int:
    try:
        return int(row.get(key, 0))
    except (TypeError, ValueError):
        return 0


def mean(rows: list[dict], key: str) -> float:
    values = [as_float(row, key) for row in rows]
    return statistics.fmean(values) if values else 0.0


def format_table(headers: list[str], rows: list[list]) -> str:
    widths = [len(str(header)) for header in headers]
    for row in rows:
        for index, value in enumerate(row):
            widths[index] = max(widths[index], len(str(value)))

    def render(row):
        return "  ".join(str(value).ljust(widths[i])
                         for i, value in enumerate(row))

    separator = ["-" * width for width in widths]
    return "\n".join([render(headers), render(separator)] +
                     [render(row) for row in rows])


def planning_report(rows: list[dict]) -> list[str]:
    for row in rows:
        row["success_bool"] = as_bool(row.get("success"))
        row["collision_bool"] = as_bool(row.get("collision_free"))

    lines = ["", "=== Planning benchmark ==="]
    grouped = defaultdict(list)
    for row in rows:
        grouped[row.get("planner", "unknown")].append(row)

    success_rows = []
    for planner, group in sorted(grouped.items()):
        success_rows.append([
            planner,
            f"{100.0 * sum(r['success_bool'] for r in group) / len(group):.1f}%",
            f"{100.0 * sum(r['collision_bool'] for r in group) / len(group):.1f}%",
            len(group),
        ])
    lines += ["", "Success and collision-free rates",
              format_table(["Planner", "Success %", "Collision %", "Runs"],
                           success_rows)]

    successful = [row for row in rows if row["success_bool"]]
    metrics = ["planning_time_ms", "path_length", "minimum_obstacle_distance",
               "turning_count", "average_curvature", "smoothness_score"]
    average_rows = []
    for planner, group in sorted(grouped.items()):
        group = [row for row in group if row["success_bool"]]
        if group:
            average_rows.append([planner] +
                                [f"{mean(group, metric):.4f}" for metric in metrics])
    if average_rows:
        lines += ["", "Successful runs: mean path metrics",
                  format_table(["Planner", *metrics], average_rows)]

    if successful:
        lines += ["", "Planning time by planner x map"]
        maps = sorted({row.get("map", "unknown") for row in successful})
        pivot = []
        for planner in sorted(grouped):
            row = [planner]
            for map_name in maps:
                matching = [
                    result for result in successful
                    if (result.get("planner") == planner
                        and result.get("map") == map_name)
                ]
                row.append(
                    f"{mean(matching, 'planning_time_ms'):.4f}"
                    if matching else "-"
                )
            pivot.append(row)
        lines.append(format_table(["Planner", *maps], pivot))
    return lines


def tracking_report(rows: list[dict]) -> list[str]:
    if not rows:
        return []
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row.get("planner", "unknown"),
                 row.get("controller", "unknown"))].append(row)
    table = []
    for (planner, controller), group in sorted(grouped.items()):
        table.append([
            planner, controller, len(group),
            f"{100.0 * sum(as_bool(r.get('goal_reached')) for r in group) / len(group):.1f}%",
            f"{mean(group, 'mean_cross_track'):.4f}",
            f"{mean(group, 'max_cross_track'):.4f}",
            f"{mean(group, 'mean_heading_error'):.4f}",
            f"{mean(group, 'max_heading_error'):.4f}",
        ])
    return ["", "=== Tracking benchmark ===", "",
            format_table(["Planner", "Controller", "Runs", "Goal %",
                          "Mean CTE", "Max CTE", "Mean HE", "Max HE"], table)]


def dynamic_report(rows: list[dict]) -> list[str]:
    if not rows:
        return []
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row.get("map", "unknown"),
                 row.get("controller", "unknown"))].append(row)
    table = []
    for (map_name, controller), group in sorted(grouped.items()):
        completion_rate = 100.0 * statistics.fmean(
            as_float(row, "frames_run")
            / max(as_float(row, "frames_requested"), 1.0)
            for row in group
        )
        table.append([
            map_name, controller, len(group),
            f"{mean(group, 'replanning_count'):.2f}",
            f"{mean(group, 'dstar_total_time_ms'):.4f}",
            f"{mean(group, 'astar_total_time_ms'):.4f}",
            f"{mean(group, 'dstar_over_astar_speedup'):.2f}",
            f"{mean(group, 'mean_replan_steering_delta'):.4f}",
            f"{completion_rate:.1f}%",
        ])
    return ["", "=== Dynamic replanning benchmark ===", "",
            format_table(["Map", "Controller", "Runs", "Replans",
                          "D* ms", "A* ms", "Speedup", "Replan steer jump",
                          "Frames complete %"], table)]


def local_planner_report(rows: list[dict], dynamic: bool = False) -> list[str]:
    """Summarize the local-planner benchmark when present."""
    if not rows:
        return []
    grouped = defaultdict(list)
    for row in rows:
        grouped[(row.get("controller", "unknown"),
                 row.get("local_planner", "unknown"))].append(row)

    table = []
    for (controller, planner), group in sorted(grouped.items()):
        success_rate = 100.0 * sum(
            as_bool(row.get("success")) for row in group) / len(group)
        goal_rate = 100.0 * sum(
            as_bool(row.get("goal_reached")) for row in group) / len(group)
        steps_key = "steps" if dynamic else "controller_trace_steps"
        time_key = ("total_dstar_replanning_time_ms"
                    if dynamic else "planning_time_ms")
        table.append([
            controller,
            planner,
            len(group),
            f"{success_rate:.1f}%",
            f"{goal_rate:.1f}%",
            f"{mean(group, steps_key):.1f}",
            f"{mean(group, 'local_planner_rollouts'):.1f}",
            f"{mean(group, time_key):.4f}",
        ])

    title = "Dynamic local-planner benchmark" if dynamic \
        else "Local-planner benchmark"
    return ["", f"=== {title} ===", "",
            format_table(["Controller", "Local planner", "Runs", "Success %",
                          "Goal %", "Mean steps", "Mean rollouts",
                          "Mean time ms"], table)]


def main() -> int:
    parser = argparse.ArgumentParser(description="Compare RobotNav benchmark CSVs")
    parser.add_argument(
        "results", nargs="?", default="autoplanner/data/demos/planning_results.csv",
        help="CSV file or benchmark output directory (default: bundled demo)"
    )
    parser.add_argument("--tracking-csv", default=None)
    parser.add_argument("--dynamic-csv", default=None)
    parser.add_argument("--output", "-o", default=None)
    args = parser.parse_args()

    root = Path(args.results)
    if root.is_dir():
        planning_path = root / "planning_results.csv"
        tracking_path = root / "tracking_results.csv"
        dynamic_path = root / "dynamic_replanning_results.csv"
        local_planner_path = root / "local_planner_results.csv"
        dynamic_local_planner_path = root / "dynamic_local_planner_results.csv"
    else:
        planning_path = root
        tracking_path = Path(args.tracking_csv) if args.tracking_csv else None
        dynamic_path = Path(args.dynamic_csv) if args.dynamic_csv else None
        local_planner_path = None
        dynamic_local_planner_path = None

    lines = ["=" * 100, "ROBOTNAV BENCHMARK SUMMARY", "=" * 100]
    lines += planning_report(load_rows(planning_path)
                             if planning_path and planning_path.exists() else [])
    lines += tracking_report(
        load_rows(tracking_path)
        if tracking_path and tracking_path.exists() else [])
    lines += dynamic_report(
        load_rows(dynamic_path)
        if dynamic_path and dynamic_path.exists() else [])
    lines += local_planner_report(
        load_rows(local_planner_path)
        if local_planner_path and local_planner_path.exists() else [])
    lines += local_planner_report(
        load_rows(dynamic_local_planner_path)
        if dynamic_local_planner_path and dynamic_local_planner_path.exists()
        else [],
        dynamic=True)
    lines += ["", "=" * 100]
    report = "\n".join(lines)
    print(report)
    if args.output:
        Path(args.output).write_text(report + "\n")
        print(f"\nSummary saved to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
