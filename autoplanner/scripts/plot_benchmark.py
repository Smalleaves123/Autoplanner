#!/usr/bin/env python3
"""Plot benchmark comparison charts from benchmark CSV."""
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv',
                    default='autoplanner/data/demos/planning_results.csv',
                    help='benchmark CSV file (default: bundled demo result)')
    ap.add_argument('--output_dir',
                    default='autoplanner/results/benchmark/demo_images')
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df_ok = df[df['success'] == True]

    import os
    os.makedirs(args.output_dir, exist_ok=True)

    time_column = 'planning_time_ms' if 'planning_time_ms' in df.columns else 'time_ms'
    metric_candidates = [
        (time_column, 'Planning Time (ms)'),
        ('path_length', 'Path Length'),
        ('minimum_obstacle_distance', 'Minimum Obstacle Distance'),
        ('turning_count', 'Turning Count'),
        ('average_curvature', 'Average Curvature'),
        ('smoothness_score', 'Smoothness Score'),
    ]
    metrics = [(column, title) for column, title in metric_candidates
               if column in df.columns]

    columns = min(3, max(1, len(metrics)))
    rows = int(np.ceil(len(metrics) / columns))
    fig, axes = plt.subplots(rows, columns,
                             figsize=(6 * columns, 5 * rows), squeeze=False)
    axes = axes.ravel()
    for ax, (col, title) in zip(axes, metrics):
        piv = df_ok.pivot_table(values=col, index='planner',
                                 columns='map', aggfunc='mean')
        piv.plot(kind='bar', ax=ax)
        ax.set_title(title)
        ax.set_xlabel('')
        ax.tick_params(axis='x', rotation=45)
        ax.legend(title='', fontsize=8)
    for ax in axes[len(metrics):]:
        ax.remove()
    plt.tight_layout()
    plt.savefig(f'{args.output_dir}/benchmark_compare.png', dpi=150)
    print(f'Saved: {args.output_dir}/benchmark_compare.png')

if __name__ == '__main__':
    main()
