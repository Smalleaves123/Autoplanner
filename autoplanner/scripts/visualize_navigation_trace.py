#!/usr/bin/env python3
"""Visualize RobotNav pipeline trace CSV files over an occupancy grid."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_map(path):
    rows = []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                rows.append([1 if char in "1#@" else 0 for char in line])
    if not rows or any(len(row) != len(rows[0]) for row in rows):
        raise ValueError(f"map must contain non-empty rectangular rows: {path}")
    return np.array(rows, dtype=int)


def parse_float(row, key, default=np.nan):
    value = row.get(key, "")
    if value == "":
        return default
    return float(value)


def load_trace(path):
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    if not rows:
        return {
            "x": np.array([]),
            "y": np.array([]),
            "time": np.array([]),
            "cross_track_error": np.array([]),
            "replanned": np.array([], dtype=bool),
            "obstacle_x": np.array([]),
            "obstacle_y": np.array([]),
        }

    return {
        "x": np.array([parse_float(row, "x") for row in rows]),
        "y": np.array([parse_float(row, "y") for row in rows]),
        "time": np.array(
            [parse_float(row, "time", index) for index, row in enumerate(rows)]
        ),
        "cross_track_error": np.array(
            [parse_float(row, "cross_track_error") for row in rows]
        ),
        "replanned": np.array(
            [row.get("replanned", "0") in {"1", "true", "True"} for row in rows]
        ),
        "obstacle_x": np.array([parse_float(row, "obstacle_x") for row in rows]),
        "obstacle_y": np.array([parse_float(row, "obstacle_y") for row in rows]),
    }


def load_metrics(path):
    if not path:
        return {}
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def load_path(path):
    if not path:
        return np.array([]), np.array([])
    points = []
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        for row in reader:
            if len(row) < 2:
                continue
            try:
                points.append((float(row[0]), float(row[1])))
            except ValueError:
                # Skip a CSV header or other non-numeric metadata row.
                continue
    if not points:
        return np.array([]), np.array([])
    values = np.asarray(points, dtype=float)
    return values[:, 0], values[:, 1]


def finite_points(x_values, y_values):
    mask = (
        np.isfinite(x_values)
        & np.isfinite(y_values)
        & (x_values >= 0)
        & (y_values >= 0)
    )
    return x_values[mask], y_values[mask]


def metric_summary(metrics):
    if not metrics:
        return ""
    parts = []
    for key in (
        "status_code",
        "goal_reached",
        "replanning_count",
        "external_update_count",
        "moving_obstacle_update_count",
        "collision_steps",
        "safe_stop",
        "steps",
    ):
        if key in metrics:
            parts.append(f"{key}: {metrics[key]}")
    if "controller_trace_steps" in metrics:
        parts.append(f"steps: {metrics['controller_trace_steps']}")
    return " | ".join(parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True, help="occupancy grid txt map")
    parser.add_argument(
        "--trace", required=True, help="trace.csv from a RobotNav pipeline"
    )
    parser.add_argument(
        "--path", help="optional planned path CSV to overlay on the map"
    )
    parser.add_argument("--metrics", help="optional metrics.json")
    parser.add_argument("--output", default="navigation_trace.png")
    parser.add_argument("--title", default="RobotNav Trace")
    args = parser.parse_args()

    grid = load_map(args.map)
    trace = load_trace(args.trace)
    metrics = load_metrics(args.metrics)
    planned_x, planned_y = load_path(args.path)

    fig, (map_ax, error_ax) = plt.subplots(
        1, 2, figsize=(13, 6), gridspec_kw={"width_ratios": [1.25, 1.0]}
    )
    map_ax.imshow(grid, cmap="gray_r", origin="upper", interpolation="none")

    if planned_x.size:
        map_ax.plot(
            planned_x,
            planned_y,
            color="#1f77b4",
            linestyle="--",
            linewidth=1.4,
            label="planned path",
            zorder=2,
        )

    if trace["x"].size:
        map_ax.plot(
            trace["x"],
            trace["y"],
            color="#d62728",
            linewidth=1.8,
            label="state trace",
        )
        map_ax.scatter(
            trace["x"][0],
            trace["y"][0],
            color="#2ca02c",
            s=48,
            label="start",
            zorder=4,
        )
        map_ax.scatter(
            trace["x"][-1],
            trace["y"][-1],
            color="#1f77b4",
            s=48,
            label="last",
            zorder=4,
        )

        replan_mask = trace["replanned"]
        if replan_mask.any():
            map_ax.scatter(
                trace["x"][replan_mask],
                trace["y"][replan_mask],
                marker="x",
                color="#ff7f0e",
                s=70,
                label="replan",
                zorder=5,
            )

        obstacle_x, obstacle_y = finite_points(
            trace["obstacle_x"], trace["obstacle_y"]
        )
        if obstacle_x.size:
            map_ax.scatter(
                obstacle_x,
                obstacle_y,
                marker="s",
                facecolors="none",
                edgecolors="#9467bd",
                s=64,
                label="obstacle update",
                zorder=4,
            )

        valid_error = np.isfinite(trace["cross_track_error"])
        if valid_error.any():
            error_ax.plot(
                trace["time"][valid_error],
                trace["cross_track_error"][valid_error],
                color="#d62728",
                linewidth=1.5,
            )
            error_ax.set_ylabel("cross-track error")
        else:
            error_ax.text(0.5, 0.5, "no error samples", ha="center", va="center")

    map_ax.set_title(args.title)
    map_ax.set_xlabel("x cell")
    map_ax.set_ylabel("y cell")
    map_ax.set_aspect("equal")

    error_ax.set_title("Tracking Error")
    error_ax.set_xlabel("time")
    error_ax.grid(True, alpha=0.25)
    handles, labels = map_ax.get_legend_handles_labels()
    if handles:
        map_ax.legend(loc="best")
    summary = metric_summary(metrics)
    if summary:
        fig.suptitle(summary, fontsize=10)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(f"Saved: {output}")


if __name__ == "__main__":
    main()
