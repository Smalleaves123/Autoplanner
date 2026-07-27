#!/usr/bin/env python3
"""Visualize a planned path overlaid on the grid map."""
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def load_map(path):
    grid = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                grid.append([1 if c in '1#@' else 0 for c in line])
    return np.array(grid)

def load_path(path):
    pts = []
    with open(path) as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 2:
                pts.append((float(parts[0]), float(parts[1])))
    return pts

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='autoplanner/data/maps/simple_50x50.txt',
                    help='Grid map file (default: bundled simple map)')
    ap.add_argument('--path', default='autoplanner/data/demos/planned_path.csv',
                    help='Planner path CSV (default: bundled demo path)')
    ap.add_argument('--output',
                    default='autoplanner/results/images/path_visualization.png')
    args = ap.parse_args()

    grid = load_map(args.map)
    path = load_path(args.path)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap='gray_r', origin='upper', interpolation='none')

    if path:
        xs, ys = zip(*path)
        ax.plot(xs, ys, 'r-', linewidth=2, label='Path')
        ax.plot(xs[0], ys[0], 'go', markersize=8, label='Start')
        ax.plot(xs[-1], ys[-1], 'bo', markersize=8, label='Goal')

    ax.legend()
    ax.set_title('Path Visualization')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output, dpi=150)
    print(f'Saved: {output}')

if __name__ == '__main__':
    main()
