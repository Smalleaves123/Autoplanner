#!/usr/bin/env python3
"""Plot benchmark comparison charts from benchmark CSV."""
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True, help='benchmark CSV file')
    ap.add_argument('--output_dir', default='results/images')
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    df_ok = df[df['success'] == True]

    import os
    os.makedirs(args.output_dir, exist_ok=True)

    metrics = [
        ('time_ms', 'Planning Time (ms)'),
        ('path_length', 'Path Length'),
        ('expanded_nodes', 'Expanded Nodes'),
    ]

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for ax, (col, title) in zip(axes, metrics):
        piv = df_ok.pivot_table(values=col, index='planner',
                                 columns='map', aggfunc='mean')
        piv.plot(kind='bar', ax=ax)
        ax.set_title(title)
        ax.set_xlabel('')
        ax.tick_params(axis='x', rotation=45)
        ax.legend(title='', fontsize=8)
    plt.tight_layout()
    plt.savefig(f'{args.output_dir}/benchmark_compare.png', dpi=150)
    print(f'Saved: {args.output_dir}/benchmark_compare.png')

if __name__ == '__main__':
    main()
