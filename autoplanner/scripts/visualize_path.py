#!/usr/bin/env python3
"""Visualize a planned path overlaid on the grid map."""
import argparse
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
    ap.add_argument('--map', required=True)
    ap.add_argument('--path', required=True)
    ap.add_argument('--output', default='path_vis.png')
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
    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f'Saved: {args.output}')

if __name__ == '__main__':
    main()
