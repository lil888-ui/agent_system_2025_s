#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv, math, sys

THRESHOLD_RAD = math.radians(10.0)        # 10 度 → ラジアン

def compute_angle(p_prev, p_center, p_next):
    """三つの 2D 点のなす角（p_prev→p_center と p_center→p_next）を返す"""
    v1 = (p_center[0] - p_prev[0], p_center[1] - p_prev[1])
    v2 = (p_next[0] - p_center[0], p_next[1] - p_center[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)

    # どちらかのベクトル長がゼロなら「角度 0」（＝ほぼ直線扱い）で返す
    if n1 == 0 or n2 == 0:
        return 0.0

    cosang = max(-1.0, min(1.0, (v1[0]*v2[0] + v1[1]*v2[1]) / (n1 * n2)))
    return math.acos(cosang)

def main(infile='log_after_filter_distance.csv',
         outfile='log.csv'):
    try:
        with open(infile, newline='') as fin, open(outfile, 'w', newline='') as fout:
            reader  = csv.reader(fin)
            rows    = [row for row in reader if len(row) == 4]   # 4 列以外は捨てる
            if len(rows) < 3:
                print('行数が足りません。', file=sys.stderr)
                return

            filtered = [rows[0], rows[1]]        # 先頭 2 行はそのまま
            for row in rows[2:]:
                p1 = (float(filtered[-2][1]), float(filtered[-2][2]))
                p2 = (float(filtered[-1][1]), float(filtered[-1][2]))
                p3 = (float(row[1]),          float(row[2]))

                if compute_angle(p1, p2, p3) > THRESHOLD_RAD:
                    filtered.append(row)
                # 角度 ≤ 10° ならスキップ

            # ゴール点が残っていなければ必ず追加
            if filtered[-1] != rows[-1]:
                filtered.append(rows[-1])

            csv.writer(fout).writerows(filtered)

    except FileNotFoundError:
        print(f'ファイルが見つかりません: {infile}', file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
