#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv, math, sys

THRESHOLD_DEG = 10.0
THRESHOLD_RAD = math.radians(THRESHOLD_DEG)

def compute_angle(p_prev, p_center, p_next):
    # 正しいベクトル：中点−前点 と 次点−中点
    v1 = (p_center[0] - p_prev[0], p_center[1] - p_prev[1])
    v2 = (p_next[0]   - p_center[0], p_next[1]   - p_center[1])
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    n1 = math.hypot(*v1)
    n2 = math.hypot(*v2)
    if n1==0 or n2==0:
        return 0.0
    cosang = max(-1.0, min(1.0, dot / (n1 * n2)))
    return math.acos(cosang)

def main():
    infile  = 'log_after_filter_distance.csv'
    outfile = 'log.csv'

    try:
        with open(infile,  newline='') as fin, \
             open(outfile, 'w', newline='') as fout:

            reader = csv.reader(fin)
            rows = [row for row in reader if len(row)>=3]
            if len(rows) < 3:
                print('行数が足りません。', file=sys.stderr)
                return

            writer = csv.writer(fout)
            # 最初の２行はそのまま残す
            filtered = [rows[0], rows[1]]

            for row in rows[2:]:
                p1 = (float(filtered[-2][1]), float(filtered[-2][2]))
                p2 = (float(filtered[-1][1]), float(filtered[-1][2]))
                p3 = (float(row[1]),           float(row[2]))
                if compute_angle(p1, p2, p3) > THRESHOLD_RAD:
                    filtered.append(row)
                # そうでなければスキップ

            for row in filtered:
                writer.writerow(row)

    except FileNotFoundError:
        print(f'ファイルが見つかりません: {infile}', file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
