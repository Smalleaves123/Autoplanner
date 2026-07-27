#!/usr/bin/env python3
"""Visualize a costmap — requires the costmap data exported from C++."""
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default='autoplanner/data/maps/simple_50x50.txt',
                    help='Original grid map (.txt; default: bundled simple map)')
    ap.add_argument('--output',
                    default='autoplanner/results/images/costmap.png')
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
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    plt.tight_layout()
    plt.savefig(output, dpi=150)
    print(f'Saved: {output}')

if __name__ == '__main__':
    main()
