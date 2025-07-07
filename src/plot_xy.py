#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_xy.py  : CSV 内の (x,y) をそのままプロットするだけ
使い方:
    python3 plot_xy.py path/to/log.csv
オプション:
    --line  連結線を引く（デフォルトは点だけ）
"""
import csv, argparse
import matplotlib.pyplot as plt

def read_xy(csv_path):
    xs, ys = [], []
    with open(csv_path, newline='') as f:
        for row in csv.reader(f):
            if len(row) < 3:     # 時刻,x,y,... の4列前提だが y までは最低欲しい
                continue
            xs.append(float(row[1]))
            ys.append(float(row[2]))
    if not xs:
        raise RuntimeError("有効な行が 0 でした")
    return xs, ys

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv_file")
    ap.add_argument("--line", action="store_true", help="scatter ではなく線をつなぐ")
    args = ap.parse_args()

    xs, ys = read_xy(args.csv_file)

    if args.line:
        plt.plot(xs, ys)
    else:
        plt.scatter(xs, ys, s=5)     # s=マーカーサイズ
    plt.axis("equal")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(args.csv_file)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
