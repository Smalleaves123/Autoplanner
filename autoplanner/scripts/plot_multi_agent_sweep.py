#!/usr/bin/env python3
"""Plot aggregate metrics from ``multi_agent_sweep.py`` output."""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path


def _aggregate(rows: list[dict[str, object]]) -> dict[tuple[str, int, int], dict[str, float]]:
    grouped: dict[tuple[str, int, int], list[dict[str, object]]] = defaultdict(list)
    for row in rows:
        key = (str(row["algorithm"]), int(row["agents"]), int(row["max_cbs_nodes"]))
        grouped[key].append(row)
    result = {}
    for key, values in grouped.items():
        result[key] = {
            metric: sum(float(value[metric]) for value in values) / len(values)
            for metric in (
                "success_rate",
                "mean_successful_makespan",
                "mean_runtime_ms",
                "mean_vertex_conflicts",
            )
        }
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="multi_agent_sweep.json")
    parser.add_argument("--output", required=True, help="output PNG path")
    args = parser.parse_args(argv)

    import matplotlib.pyplot as plt

    source = Path(args.input).expanduser()
    data = json.loads(source.read_text(encoding="utf-8"))
    aggregate = _aggregate(data["runs"])
    if not aggregate:
        raise SystemExit("sweep JSON contains no runs")

    metrics = (
        ("success_rate", "success rate", "fraction"),
        ("mean_successful_makespan", "successful makespan", "steps"),
        ("mean_runtime_ms", "planning runtime", "milliseconds"),
        ("mean_vertex_conflicts", "vertex conflicts", "count"),
    )
    figure, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    colors = {"prioritized": "#4472c4", "cbs": "#ed7d31"}
    markers = {"prioritized": "o", "cbs": "s"}
    shown_labels: set[str] = set()
    for axis, (metric, title, unit) in zip(axes.flat, metrics):
        for algorithm in ("prioritized", "cbs"):
            budgets = sorted({key[2] for key in aggregate if key[0] == algorithm})
            for budget in budgets:
                points = sorted(
                    (key[1], values[metric])
                    for key, values in aggregate.items()
                    if key[0] == algorithm and key[2] == budget)
                if not points:
                    continue
                label = algorithm if algorithm == "prioritized" else f"cbs ({budget} nodes)"
                if label in shown_labels:
                    label = "_nolegend_"
                else:
                    shown_labels.add(label)
                axis.plot(
                    [point[0] for point in points],
                    [point[1] for point in points],
                    color=colors[algorithm],
                    marker=markers[algorithm],
                    linewidth=1.8,
                    label=label,
                    alpha=0.9 if algorithm == "cbs" else 0.65,
                )
        axis.set_title(title)
        axis.set_xlabel("agents")
        axis.set_ylabel(unit)
        axis.grid(alpha=0.25)
        axis.set_xticks(sorted({key[1] for key in aggregate}))
    axes[0, 0].set_ylim(0.0, 1.05)
    axes[0, 0].legend(fontsize="small")
    figure.suptitle("Multi-agent solver sweep")
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
