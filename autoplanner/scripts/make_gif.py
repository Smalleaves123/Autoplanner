#!/usr/bin/env python3
"""Generate an animated GIF of the search tree growing (RRT / RRT*).

Reads a tree edges CSV (x1,y1,x2,y2) produced by the RRT/RRT* examples
and creates an animation showing the tree growth from root to leaves.

Usage:
    python scripts/make_gif.py \
        --tree results/rrt_tree.csv \
        --path results/rrt_path.csv \
        --map data/maps/maze_100x100.txt \
        --output results/gifs/rrt_animation.gif
"""
import argparse
import io

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation


def load_map(path):
    grid = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                grid.append([1 if c in "1#@" else 0 for c in line])
    return np.array(grid)


def load_tree_edges(path):
    edges = []
    with open(path) as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 4:
                edges.append((
                    (float(parts[0]), float(parts[1])),
                    (float(parts[2]), float(parts[3])),
                ))
    return edges


def load_path(path):
    pts = []
    with open(path) as f:
        f.readline()
        for line in f:
            parts = line.strip().split(",")
            if len(parts) >= 2:
                pts.append((float(parts[0]), float(parts[1])))
    return pts


def main():
    ap = argparse.ArgumentParser(
        description="Generate GIF animation of tree-based planner")
    ap.add_argument("--tree", required=True, help="Tree edges CSV file")
    ap.add_argument("--path", default=None, help="Path CSV file (optional)")
    ap.add_argument("--map", default=None, help="Grid map file (optional)")
    ap.add_argument("--output", default="results/gifs/animation.gif")
    ap.add_argument("--max_frames", type=int, default=100,
                    help="Number of frames (sub-sampled)")
    ap.add_argument("--fps", type=int, default=10,
                    help="Frames per second")
    ap.add_argument("--dpi", type=int, default=100)
    args = ap.parse_args()

    edges = load_tree_edges(args.tree)
    if not edges:
        print("No tree edges found.", flush=True)
        return

    path_pts = load_path(args.path) if args.path else []
    grid = load_map(args.map) if args.map else None

    # Subsample to max_frames
    stride = max(1, len(edges) // args.max_frames)
    frames_edges = edges[::stride]

    fig, ax = plt.subplots(figsize=(8, 8))

    if grid is not None:
        ax.imshow(grid, cmap="gray_r", origin="upper", interpolation="none")

    # Find bounds
    all_x = [e[0][0] for e in edges] + [e[1][0] for e in edges]
    all_y = [e[0][1] for e in edges] + [e[1][1] for e in edges]
    margin = 2
    ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
    ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
    ax.set_aspect("equal")
    ax.set_title("RRT Tree Growth")

    line, = ax.plot([], [], "b-", alpha=0.4, linewidth=0.5)
    path_line, = ax.plot([], [], "r-", linewidth=2)

    # Start point
    if edges:
        ax.plot(edges[0][0][0], edges[0][0][1], "go", markersize=8)
    # Goal point (last node of first path)
    if path_pts:
        ax.plot(path_pts[-1][0], path_pts[-1][1], "bo", markersize=8)

    def update(frame_idx):
        current_edges = frames_edges[: frame_idx + 1]
        xs = []
        ys = []
        for (x1, y1), (x2, y2) in current_edges:
            xs.extend([x1, x2, None])
            ys.extend([y1, y2, None])
        line.set_data(xs, ys)

        # Show path if it exists and tree has grown enough
        if path_pts and frame_idx >= len(frames_edges) * 0.5:
            px = [p[0] for p in path_pts]
            py = [p[1] for p in path_pts]
            path_line.set_data(px, py)
        else:
            path_line.set_data([], [])

        return line, path_line

    ani = FuncAnimation(
        fig, update, frames=len(frames_edges),
        interval=1000 / args.fps, blit=True, repeat=True
    )

    try:
        import os
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        ani.save(args.output, writer="pillow", fps=args.fps, dpi=args.dpi)
        print(f"Saved GIF to {args.output}")
    except Exception as e:
        print(f"Warning: could not save GIF ({e})", flush=True)
        print("Try installing pillow: pip install pillow")


if __name__ == "__main__":
    main()
