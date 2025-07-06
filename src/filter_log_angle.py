#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import sys

THRESHOLD_DEG = 10.0
THRESHOLD_RAD = math.radians(THRESHOLD_DEG)

def angle(p1, p2, p3):
    # p1→p2, p2→p3 の角度を返す
    v1 = (p1[0]-p2[0], p1[1]-p2[1])
    v2 = (p3[0]-p2[0], p3[1]-p2[1])
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    n1 = math.hypot(*v1)
    n2 = math.hypot(*v2)
    if n1==0 or n2==0:
        return 0.0
    cosv = max(-1, min(1, dot/(n1*n2)))
    return math.acos(cosv)

def main():
    in_path  = 'log_after_filter_distance.csv'
    out_path = 'log.csv'

    try:
        with open(in_path, newline='') as fin, open(out_path, 'w', newline='') as fout:
            reader = csv.reader(fin)
            writer = csv.writer(fout)

            # まず全行読み込み
            rows = [row for row in reader if len(row)>=3]
            if len(rows) < 3:
                print("行数が足りません。")
                return

            # フィルタ済みリストに最初の２行だけ追加
            filtered = [rows[0], rows[1]]

            # 3行目以降を１行ずつチェック
            for row in rows[2:]:
                # 座標だけ取り出し
                p1 = (float(filtered[-2][1]), float(filtered[-2][2]))
                p2 = (float(filtered[-1][1]), float(filtered[-1][2]))
                p3 = (float(row[1]), float(row[2]))

                if angle(p1, p2, p3) > THRESHOLD_RAD:
                    filtered.append(row)
                # そうでなければスキップ

            # 結果を書き出し
            for row in filtered:
                writer.writerow(row)

    except FileNotFoundError:
        print(f"ファイルが見つかりません: {in_path}", file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
