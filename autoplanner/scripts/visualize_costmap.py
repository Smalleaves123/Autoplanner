#!/usr/bin/env python3
"""Visualize a costmap — requires the costmap data exported from C++."""
import argparse
import matplotlib.pyplot as plt
import numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True,
                    help='Original grid map (.txt)')
    ap.add_argument('--output', default='costmap.png')
    args = ap.parse_args()

    grid = []
    with open(args.map) as f:
        for line in f:
            line = line.strip()
            if line:
                grid.append([1 if c in '1#@' else 0 for c in line])
    grid = np.array(grid)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap='hot', origin='upper', interpolation='none')
    ax.set_title('Costmap (raw occupancy)')
    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f'Saved: {args.output}')

if __name__ == '__main__':
    main()
