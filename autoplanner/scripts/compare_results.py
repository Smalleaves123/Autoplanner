#!/usr/bin/env python3
"""Compare benchmark results across planners with summary statistics.

Usage:
    python scripts/compare_results.py results/benchmark/all_results.csv
"""
import argparse
import sys

import pandas as pd


def main():
    ap = argparse.ArgumentParser(
        description="Summarize and compare benchmark CSV results")
    ap.add_argument("csv_path", help="Path to benchmark CSV file")
    ap.add_argument("--output", "-o", default=None,
                    help="Save summary to file (optional)")
    args = ap.parse_args()

    try:
        df = pd.read_csv(args.csv_path)
    except FileNotFoundError:
        print(f"File not found: {args.csv_path}", file=sys.stderr)
        sys.exit(1)

    print("=" * 70)
    print("BENCHMARK SUMMARY")
    print("=" * 70)

    # Success rate by planner
    print("\n--- Success Rate by Planner ---")
    success_rate = df.groupby("planner")["success"].agg(["mean", "count"])
    success_rate["mean"] = success_rate["mean"] * 100
    success_rate.columns = ["Success %", "Runs"]
    print(success_rate.to_string(float_format="%.1f"))

    # Average metrics by planner (successful runs only)
    ok = df[df["success"] == True]
    if not ok.empty:
        print("\n--- Average Metrics (successful runs only) ---")
        metrics = ["time_ms", "path_length", "expanded_nodes"]
        available = [m for m in metrics if m in ok.columns]
        if available:
            summary = ok.groupby("planner")[available].mean()
            print(summary.to_string(float_format="%.3f"))

    # By map
    print("\n--- Success Rate by Map ---")
    map_rate = df.groupby("map")["success"].agg(["mean", "count"])
    map_rate["mean"] = map_rate["mean"] * 100
    map_rate.columns = ["Success %", "Runs"]
    print(map_rate.to_string(float_format="%.1f"))

    # Planner × Map pivot
    if "time_ms" in df.columns:
        print("\n--- Planning Time (ms) by Planner × Map ---")
        piv = ok.pivot_table(values="time_ms", index="planner",
                             columns="map", aggfunc="mean")
        print(piv.to_string(float_format="%.2f"))

        print("\n--- Path Length by Planner × Map ---")
        piv_len = ok.pivot_table(values="path_length", index="planner",
                                 columns="map", aggfunc="mean")
        print(piv_len.to_string(float_format="%.2f"))

    print("=" * 70)

    # Optionally save
    if args.output:
        with open(args.output, "w") as f:
            f.write("Success Rate by Planner\n")
            f.write(success_rate.to_string())
            if not ok.empty:
                f.write("\n\nAverage Metrics\n")
                f.write(summary.to_string())
        print(f"\nSummary saved to {args.output}")


if __name__ == "__main__":
    main()
