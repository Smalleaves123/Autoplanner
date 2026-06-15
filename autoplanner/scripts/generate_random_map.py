#!/usr/bin/env python3
"""Generate a random obstacle map."""
import argparse
import random

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--width', type=int, default=100)
    ap.add_argument('--height', type=int, default=100)
    ap.add_argument('--density', type=float, default=0.2)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()

    random.seed(args.seed)
    w, h = args.width, args.height

    with open(args.output, 'w') as f:
        for y in range(h):
            row = []
            for x in range(w):
                if x == 0 or y == 0 or x == w-1 or y == h-1:
                    row.append('1')
                else:
                    row.append('1' if random.random() < args.density else '0')
            # Ensure start/goal areas free
            if y <= 2 or y >= h-3:
                for x in range(1, min(3, w-1)):
                    row[x] = '0'
                for x in range(max(1, w-3), w-1):
                    row[x] = '0'
            f.write(''.join(row) + '\n')
    print(f'Generated {w}x{h} map with {args.density:.0%} density: {args.output}')

if __name__ == '__main__':
    main()
