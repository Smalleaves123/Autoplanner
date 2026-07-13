#!/usr/bin/env python3
"""Compare benchmark results using only Python's standard library.

Usage:
    python scripts/compare_results.py results/benchmark/all_results.csv
"""
import argparse
import csv
import statistics
import sys
from collections import defaultdict


METRICS = ("time_ms", "path_length", "expanded_nodes")


def load_rows(path):
    try:
        with open(path, newline="") as f:
            rows = list(csv.DictReader(f))
    except FileNotFoundError:
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    for row in rows:
        row["success"] = row.get("success", "").lower() in ("1", "true", "yes")
        for metric in METRICS:
            try:
                row[metric] = float(row.get(metric, 0.0))
            except (TypeError, ValueError):
                row[metric] = 0.0
    return rows


def group_by(rows, key):
    groups = defaultdict(list)
    for row in rows:
        groups[row.get(key, "unknown")].append(row)
    return groups


def mean(rows, metric):
    values = [row[metric] for row in rows]
    return statistics.fmean(values) if values else 0.0


def format_table(headers, table):
    widths = [len(str(header)) for header in headers]
    for row in table:
        for i, value in enumerate(row):
            widths[i] = max(widths[i], len(str(value)))

    def render(row):
        return "  ".join(str(value).ljust(widths[i]) for i, value in enumerate(row))

    return "\n".join([render(headers), render(["-" * width for width in widths])]
                     + [render(row) for row in table])


def build_report(rows):
    lines = ["=" * 70, "BENCHMARK SUMMARY", "=" * 70]

    planner_groups = group_by(rows, "planner")
    success_table = []
    for planner, group in sorted(planner_groups.items()):
        success_table.append([
            planner,
            f"{100.0 * sum(row['success'] for row in group) / len(group):.1f}%",
            len(group),
        ])
    lines += ["", "--- Success Rate by Planner ---",
              format_table(["Planner", "Success %", "Runs"], success_table)]

    ok = [row for row in rows if row["success"]]
    if ok:
        avg_table = []
        for planner, group in sorted(group_by(ok, "planner").items()):
            avg_table.append([planner] + [f"{mean(group, metric):.3f}" for metric in METRICS])
        lines += ["", "--- Average Metrics (successful runs only) ---",
                  format_table(["Planner", *METRICS], avg_table)]

        lines += ["", "--- Planning Time (ms) by Planner x Map ---"]
        lines.append(format_pivot(ok, "time_ms"))
        lines += ["", "--- Path Length by Planner x Map ---"]
        lines.append(format_pivot(ok, "path_length"))

    lines.append("=" * 70)
    return "\n".join(lines)


def format_pivot(rows, metric):
    maps = sorted({row.get("map", "unknown") for row in rows})
    planners = sorted({row.get("planner", "unknown") for row in rows})
    table = []
    for planner in planners:
        values = []
        for map_name in maps:
            selected = [row for row in rows
                        if row.get("planner") == planner and row.get("map") == map_name]
            values.append(f"{mean(selected, metric):.3f}" if selected else "-")
        table.append([planner, *values])
    return format_table(["Planner", *maps], table)


def main():
    parser = argparse.ArgumentParser(description="Summarize benchmark CSV results")
    parser.add_argument("csv_path", help="benchmark CSV file")
    parser.add_argument("--output", "-o", default=None,
                        help="save the text summary to a file")
    args = parser.parse_args()

    report = build_report(load_rows(args.csv_path))
    print(report)
    if args.output:
        with open(args.output, "w") as f:
            f.write(report + "\n")
        print(f"\nSummary saved to {args.output}")


if __name__ == "__main__":
    main()
