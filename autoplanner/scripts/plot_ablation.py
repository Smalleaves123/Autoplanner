#!/usr/bin/env python3
"""Plot ablation study results with grouped bars."""
import argparse, os, pandas as pd, matplotlib.pyplot as plt, numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='results/benchmark/ablation.csv')
    ap.add_argument('--output_dir', default='docs/images')
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    os.makedirs(args.output_dir, exist_ok=True)

    configs = ["A*", "Weighted A* (w=1.5)", "A* + obstacle", "A* + turning", "Improved A* (full)"]
    colors = ['#888888', '#66c2a5', '#fc8d62', '#8da0cb', '#e78ac3']

    metrics = [
        ('time_ms', 'Planning Time (ms)', 'lower is better'),
        ('path_length', 'Path Length', 'lower is better'),
        ('expanded_nodes', 'Expanded Nodes', 'lower is better'),
        ('turning_count', 'Turning Count', 'fewer turns = smoother'),
        ('smoothness', 'Smoothness Score', 'closer to 1.0 = straighter'),
    ]

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    axes = axes.flatten()

    for idx, (col, title, note) in enumerate(metrics):
        ax = axes[idx]
        maps = df['map'].unique()
        x = np.arange(len(maps))
        width = 0.15
        for j, cfg in enumerate(configs):
            subset = df[df['config'] == cfg]
            vals = [subset[subset['map'] == m][col].values[0] if m in subset['map'].values else 0 for m in maps]
            ax.bar(x + j*width, vals, width, label=cfg, color=colors[j])
        ax.set_title(f'{title}\n({note})', fontsize=11)
        ax.set_xticks(x + width*2)
        map_labels = [m.split('/')[-1].replace('.txt','') for m in maps]
        ax.set_xticklabels(map_labels, rotation=15, ha='right', fontsize=9)
        if idx == 0:
            ax.legend(fontsize=7, ncol=2)

    # Remove empty subplot
    if len(axes) > len(metrics):
        axes[-1].set_visible(False)

    fig.suptitle('Improved A* Ablation Study', fontsize=14, fontweight='bold')
    plt.tight_layout()
    path = os.path.join(args.output_dir, 'ablation_study.png')
    plt.savefig(path, dpi=150)
    print(f'Saved: {path}')

if __name__ == '__main__':
    main()
