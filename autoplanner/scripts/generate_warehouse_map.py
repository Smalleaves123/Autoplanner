#!/usr/bin/env python3
"""Generate a warehouse-style map with shelf aisles."""
import argparse

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--width', type=int, default=100)
    ap.add_argument('--height', type=int, default=100)
    ap.add_argument('--aisle_width', type=int, default=2)
    ap.add_argument('--output', required=True)
    args = ap.parse_args()

    w, h = args.width, args.height
    aisle = args.aisle_width

    with open(args.output, 'w') as f:
        for y in range(h):
            row = ['0'] * w
            # Border
            row[0] = row[w-1] = '1'
            if y == 0 or y == h - 1:
                row = ['1'] * w
            else:
                # Shelves every 8 rows
                shelf_row = (y % 8 >= 1 and y % 8 <= 6)
                if shelf_row:
                    gap1 = w // 3
                    gap2 = 2 * w // 3
                    for x in range(1, w-1):
                        if x % (gap1 // 2) < aisle and x < gap1:
                            continue
                        if x > gap1 and x < gap2 and (x - gap1) % ((gap2-gap1)//3) < aisle:
                            continue
                        if x > gap2 and (x - gap2) % ((w-gap2)//3) < aisle:
                            continue
                        row[x] = '1'
            f.write(''.join(row) + '\n')
    print(f'Generated warehouse map: {args.output}')

if __name__ == '__main__':
    main()
