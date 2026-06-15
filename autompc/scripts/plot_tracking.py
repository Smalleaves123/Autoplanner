#!/usr/bin/env python3
"""Plot trajectory tracking results from AutoMPC CSV output."""
import argparse, os
import matplotlib.pyplot as plt
import pandas as pd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', nargs='+', help='Tracking CSV files')
    ap.add_argument('--output', default='docs/tracking_comparison.png')
    args = ap.parse_args()

    fig, ax = plt.subplots(figsize=(8, 8))

    colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']
    for i, path in enumerate(args.csv):
        df = pd.read_csv(path)
        label = os.path.splitext(os.path.basename(path))[0]
        ax.plot(df['x_ref'], df['y_ref'], 'k--', alpha=0.3, linewidth=1,
                label='Reference' if i == 0 else '')
        ax.plot(df.get('x_actual', df['x']), df.get('y_actual', df['y']),
                color=colors[i % len(colors)], linewidth=1.5, label=label)

    ax.set_aspect('equal')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory Tracking Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    os.makedirs(os.path.dirname(args.output) or '.', exist_ok=True)
    plt.savefig(args.output, dpi=150)
    print(f'Saved: {args.output}')

if __name__ == '__main__':
    main()
