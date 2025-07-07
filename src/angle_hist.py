#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
angle_hist.py  : 折れ角ヒストグラム表示ツール
使い方:
    python3 angle_hist.py path/to/log.csv           # 図を表示
    python3 angle_hist.py path/to/log.csv --text    # ビンごと個数を出力
オプション:
    --bins N       … ヒストグラムのビン数 (default 180)
    --deg          … 度数で表示 (デフォルトは度数。ラジアン指定時は --rad)
    --rad          … ラジアンで表示
"""
import csv, math, argparse
import matplotlib.pyplot as plt
from collections import Counter

def compute_angles(rows):
    """CSV の行（4列以上）→ 折れ角[deg]リストを返す"""
    pts = [(float(r[1]), float(r[2])) for r in rows]
    angles = []
    for i in range(2, len(pts)):
        p_prev, p_cent, p_next = pts[i-2], pts[i-1], pts[i]
        v1 = (p_cent[0]-p_prev[0], p_cent[1]-p_prev[1])
        v2 = (p_next[0]-p_cent[0], p_next[1]-p_cent[1])
        n1, n2 = math.hypot(*v1), math.hypot(*v2)
        if n1*n2 == 0:            # ゼロ距離 → 角度 0 扱い
            angles.append(0.0)
            continue
        cosang = max(-1, min(1, (v1[0]*v2[0]+v1[1]*v2[1])/(n1*n2)))
        angles.append(math.degrees(math.acos(cosang)))
    return angles

def text_hist(data, bins):
    step = 180 / bins
    bucket = lambda a: int(a // step) * step
    cnt = Counter(bucket(a) for a in data)
    print(f"角度ビン(幅 {step:.1f}°) : 個数")
    for k in sorted(cnt):
        print(f"{k:6.1f} – {k+step:5.1f} : {cnt[k]}")

def main():
    ap = argparse.ArgumentParser(description="Angle histogram from CSV path data")
    ap.add_argument("csv_file")
    ap.add_argument("--bins", type=int, default=180, help="histogram bins")
    ap.add_argument("--text", action="store_true", help="print counts instead of showing plot")
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--rad", action="store_true", help="work in radians")
    group.add_argument("--deg", action="store_true", help="work in degrees (default)")
    args = ap.parse_args()

    # 読み込み
    with open(args.csv_file, newline='') as f:
        rows = [r for r in csv.reader(f) if len(r) >= 4]
    if len(rows) < 3:
        raise RuntimeError("行数が 3 未満です")

    angles_deg = compute_angles(rows)
    if args.rad:
        angles = [math.radians(a) for a in angles_deg]
        unit   = "rad"
    else:
        angles = angles_deg
        unit   = "deg"

    if args.text:
        text_hist(angles_deg, args.bins)
        return

    # プロット
    plt.hist(angles, bins=args.bins)
    plt.xlabel(f"angle [{unit}]")
    plt.ylabel("count")
    plt.title(f"Angle histogram ({len(angles)} samples)")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
