#!/usr/bin/env python3
"""Render a Cooperative A* benchmark trace as a multi-agent PNG.

The benchmark writes one long-form ``trace_<scenario>.csv`` per scenario.
This utility intentionally keeps plotting separate from planning so benchmark
runs remain usable on machines without a graphical backend.
"""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path


def _read_map(path: Path) -> list[str]:
    rows = [line.strip() for line in path.read_text(encoding="utf-8").splitlines()]
    rows = [row for row in rows if row]
    if not rows or any(set(row) - {"0", "1"} for row in rows):
        raise ValueError(f"invalid occupancy map: {path}")
    width = len(rows[0])
    if any(len(row) != width for row in rows):
        raise ValueError(f"map rows have inconsistent widths: {path}")
    return rows


def _read_trace(path: Path) -> dict[int, list[dict[str, str]]]:
    agents: dict[int, list[dict[str, str]]] = defaultdict(list)
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            agents[int(row["agent"])].append(row)
    if not agents:
        raise ValueError(f"trace is empty: {path}")
    for rows in agents.values():
        rows.sort(key=lambda row: int(row["time_step"]))
    return dict(agents)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", required=True, help="trace_<scenario>.csv")
    parser.add_argument("--map", required=True, help="occupancy map text file")
    parser.add_argument("--output", required=True, help="output PNG path")
    parser.add_argument("--title", default="Cooperative A* multi-agent trace")
    args = parser.parse_args(argv)

    # Import lazily so the benchmark itself does not require matplotlib.
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    map_rows = _read_map(Path(args.map).expanduser())
    agents = _read_trace(Path(args.trace).expanduser())
    height, width = len(map_rows), len(map_rows[0])
    occupied = [[int(cell) for cell in row] for row in map_rows]

    figure, axis = plt.subplots(figsize=(8, 8), constrained_layout=True)
    axis.imshow(
        occupied,
        cmap=ListedColormap(["white", "#30343b"]),
        origin="upper",
        interpolation="none",
        extent=(-0.5, width - 0.5, height - 0.5, -0.5),
        vmin=0,
        vmax=1,
    )
    colors = plt.get_cmap("tab10")
    for agent, rows in sorted(agents.items()):
        xs = [float(row["x"]) for row in rows]
        ys = [float(row["y"]) for row in rows]
        color = colors(agent % 10)
        axis.plot(xs, ys, color=color, linewidth=2, label=f"agent {agent}")
        axis.scatter(xs[0], ys[0], color=color, marker="o", s=55, zorder=3)
        axis.scatter(xs[-1], ys[-1], color=color, marker="X", s=65, zorder=3)
        waits = [row for row in rows if row["wait"].lower() == "true"]
        if waits:
            axis.scatter(
                [float(row["x"]) for row in waits],
                [float(row["y"]) for row in waits],
                color=color,
                marker=".",
                s=18,
                alpha=0.8,
            )
    axis.set(
        title=args.title,
        xlabel="grid x",
        ylabel="grid y",
        xlim=(-0.5, width - 0.5),
        ylim=(height - 0.5, -0.5),
        aspect="equal",
    )
    axis.grid(color="#b8bec8", linewidth=0.35, alpha=0.45)
    axis.legend(loc="upper right", fontsize="small")
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
