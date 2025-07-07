#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
angle_zoom.py : 折れ角 0–1° を高分解能ヒストグラム表示
使い方:
    python3 angle_zoom.py path/to/log.csv
"""

import csv, math, sys
from collections import Counter

STEP_DEG   = 0.05      # 1 ビンの幅 [deg]
RANGE_MAX  = 1.0       # 集計上限 [deg]

def angles_from_csv(csv_path):
    with open(csv_path, newline='') as f:
        rows = [r for r in csv.reader(f) if len(r) >= 4]
    if len(rows) < 3:
        sys.exit("行数が 3 未満です")

    pts = [(float(r[1]), float(r[2])) for r in rows]
    angles = []
    for i in range(2, len(pts)):
        p_prev, p_cent, p_next = pts[i-2], pts[i-1], pts[i]
        v1 = (p_cent[0]-p_prev[0], p_cent[1]-p_prev[1])
        v2 = (p_next[0]-p_cent[0], p_next[1]-p_cent[1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1*n2 == 0:
            continue                          # ゼロ距離は無視でも OK
        cosang = max(-1, min(1, (v1[0]*v2[0]+v1[1]*v2[1])/(n1*n2)))
        deg    = math.degrees(math.acos(cosang))
        if deg < RANGE_MAX:
            angles.append(deg)
    return angles

def main(csv_path):
    ang = angles_from_csv(csv_path)
    step = STEP_DEG
    bucket = lambda a: int(a // step) * step
    cnt = Counter(bucket(a) for a in ang)

    print(f"0–{RANGE_MAX}° ヒストグラム  (bin={step}°) : {len(ang)} samples")
    for i in range(int(RANGE_MAX/step)):
        lo   = i * step
        hi   = lo + step
        print(f"{lo:5.2f} – {hi:5.2f} : {cnt[lo]}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python3 angle_zoom.py log.csv")
    main(sys.argv[1])
