#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import sys

# 閾値（XY平面での距離）
THRESHOLD = 0.05

def main():
    infile_path = 'log_anntei.csv'
    outfile_path = 'log.csv'

    try:
        with open(infile_path, 'r', newline='') as fin, \
             open(outfile_path, 'w', newline='') as fout:

            reader = csv.reader(fin)
            writer = csv.writer(fout)

            # 最初の行（A）を読み込み、必ず出力
            try:
                first = next(reader)
            except StopIteration:
                print('入力ファイルが空です。')
                return

            writer.writerow(first)
            last_x = float(first[1])
            last_y = float(first[2])

            # 残りの各行を順番に処理
            for row in reader:
                if len(row) < 3:
                    # 列数不足はスキップ
                    continue

                x = float(row[1])
                y = float(row[2])

                # XY平面距離を計算
                dist = math.hypot(x - last_x, y - last_y)

                if dist >= THRESHOLD:
                    # 閾値以上なら出力＆基準点を更新
                    writer.writerow(row)
                    last_x, last_y = x, y
                # 閾値未満ならスキップ（そのままlast_x,last_yは変えない）

    except FileNotFoundError:
        print(f'ファイルが見つかりません: {infile_path}')
        sys.exit(1)
    except Exception as e:
        print(f'エラーが発生しました: {e}')
        sys.exit(1)

if __name__ == '__main__':
    main()
