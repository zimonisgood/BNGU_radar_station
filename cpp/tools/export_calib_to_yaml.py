#!/usr/bin/env python3
"""Export arrays_test_*.npy (2,3,3) to YAML for C++ loader."""
import argparse
import numpy as np

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npy')
    ap.add_argument('-o', '--out', required=True)
    args = ap.parse_args()
    arr = np.load(args.npy)
    assert arr.shape == (2, 3, 3), arr.shape
    def fmt(m):
        lines = []
        for r in m:
            lines.append('  - [' + ', '.join(f'{x:.16g}' for x in r) + ']')
        return '\n'.join(lines)
    text = f"M_ground:\n{fmt(arr[0])}\nM_height_r:\n{fmt(arr[1])}\n"
    open(args.out, 'w').write(text)
    print('wrote', args.out)

if __name__ == '__main__':
    main()
