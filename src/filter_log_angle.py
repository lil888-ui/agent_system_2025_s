#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
distance-angle_filter.py

● 役割
    1. CSV (time,x,y,z) を読み込み
    2. 「区間長 ≥ MAX_SEG_LEN」 か 「折れ角 ≥ THRESHOLD_RAD」の
       どちらかを満たす点だけを残す
    3. 先頭 2 行と末尾行は必ず残す
"""

import csv, math, sys

THRESHOLD_RAD = math.radians(0.35)   # ≈ 0.0061 rad (0.35°)
MAX_SEG_LEN   = 0.30                 # 30 cm 以上離れたら必ず採用

def compute_angle(p_prev, p_center, p_next):
    """三つの 2D 点が作る外角 [rad] を返す"""
    v1 = (p_center[0] - p_prev[0], p_center[1] - p_prev[1])
    v2 = (p_next[0]   - p_center[0], p_next[1]   - p_center[1])
    n1, n2 = math.hypot(*v1), math.hypot(*v2)
    if n1 == 0 or n2 == 0:           # 同一点が続く場合は「0°」扱い
        return 0.0
    cosang = max(-1.0, min(1.0, (v1[0]*v2[0] + v1[1]*v2[1]) / (n1 * n2)))
    return math.acos(cosang)

def main(infile='log_after_filter_distance.csv', outfile='log.csv'):
    try:
        with open(infile, newline='') as fin, open(outfile, 'w', newline='') as fout:
            reader = csv.reader(fin)
            rows   = [r for r in reader if len(r) == 4]   # 4 列以外はスキップ
            if len(rows) < 3:
                print('行数が足りません。', file=sys.stderr)
                return

            filtered = [rows[0], rows[1]]                 # 先頭 2 行そのまま
            p_prev, p_curr = (float(rows[0][1]), float(rows[0][2])), \
                             (float(rows[1][1]), float(rows[1][2]))

            for row in rows[2:]:
                p_next = (float(row[1]), float(row[2]))

                # ➊ 区間長チェック
                if math.hypot(p_next[0]-p_curr[0], p_next[1]-p_curr[1]) >= MAX_SEG_LEN:
                    filtered.append(row)
                    p_prev, p_curr = p_curr, p_next
                    continue

                # ➋ 角度チェック
                if compute_angle(p_prev, p_curr, p_next) >= THRESHOLD_RAD:
                    filtered.append(row)
                    p_prev, p_curr = p_curr, p_next
                # しきい値未満ならスキップ

            # 末尾行が残っていなければ追加
            if filtered[-1] != rows[-1]:
                filtered.append(rows[-1])

            csv.writer(fout).writerows(filtered)

    except FileNotFoundError:
        print(f'ファイルが見つかりません: {infile}', file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
