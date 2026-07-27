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


def first_available_float(row, keys, default=np.nan):
    for key in keys:
        value = row.get(key, "")
        if value != "":
            return float(value)
    return default


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
            "reference_x": np.array([]),
            "reference_y": np.array([]),
            "has_time": False,
        }

    return {
        "x": np.array(
            [first_available_float(row, ("x", "x_actual")) for row in rows]
        ),
        "y": np.array(
            [first_available_float(row, ("y", "y_actual")) for row in rows]
        ),
        "time": np.array(
            [parse_float(row, "time", index) for index, row in enumerate(rows)]
        ),
        "cross_track_error": np.array(
            [
                first_available_float(row, ("cross_track_error", "cross_track"))
                for row in rows
            ]
        ),
        "replanned": np.array(
            [row.get("replanned", "0") in {"1", "true", "True"} for row in rows]
        ),
        "obstacle_x": np.array([parse_float(row, "obstacle_x") for row in rows]),
        "obstacle_y": np.array([parse_float(row, "obstacle_y") for row in rows]),
        "reference_x": np.array([parse_float(row, "x_ref") for row in rows]),
        "reference_y": np.array([parse_float(row, "y_ref") for row in rows]),
        "has_time": "time" in rows[0],
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


def infer_cross_track_error(trace):
    errors = trace["cross_track_error"].copy()
    missing = ~np.isfinite(errors)
    if not missing.any():
        return errors
    actual_x, actual_y = trace["x"], trace["y"]
    reference_x, reference_y = trace["reference_x"], trace["reference_y"]
    valid_reference = (
        np.isfinite(actual_x)
        & np.isfinite(actual_y)
        & np.isfinite(reference_x)
        & np.isfinite(reference_y)
    )
    errors[missing & valid_reference] = np.hypot(
        actual_x[missing & valid_reference] - reference_x[missing & valid_reference],
        actual_y[missing & valid_reference] - reference_y[missing & valid_reference],
    )
    return errors


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
    parser = argparse.ArgumentParser(
        description="Render a navigation trace; runs on the bundled physics demo by default."
    )
    parser.add_argument(
        "--map", default="autoplanner/data/maps/simple_50x50.txt",
        help="occupancy grid txt map"
    )
    parser.add_argument(
        "--trace",
        default="autoplanner/data/demos/navigation_trace.csv",
        help="trace.csv from a RobotNav pipeline"
    )
    parser.add_argument(
        "--path", default="autoplanner/data/demos/planned_path.csv",
        help="optional planned path CSV to overlay on the map"
    )
    parser.add_argument(
        "--metrics",
        default="autoplanner/data/demos/navigation_metrics.json",
        help="optional metrics.json"
    )
    parser.add_argument(
        "--output",
        default="autoplanner/results/physics_dynamic/navigation_trace.png"
    )
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

    reference_x, reference_y = finite_points(
        trace["reference_x"], trace["reference_y"]
    )
    if reference_x.size:
        map_ax.plot(
            reference_x,
            reference_y,
            color="#2ca02c",
            linestyle=":",
            linewidth=1.4,
            label="reference trajectory",
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

        cross_track_error = infer_cross_track_error(trace)
        valid_error = np.isfinite(cross_track_error)
        if valid_error.any():
            error_ax.plot(
                trace["time"][valid_error],
                cross_track_error[valid_error],
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
    error_ax.set_xlabel("time" if trace["has_time"] else "sample")
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
