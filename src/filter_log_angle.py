#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import sys

# 閾値：この角度以下なら中間点をスキップ（度⇨ラジアン変換）
ANGLE_THRESHOLD_DEG = 10.0
ANGLE_THRESHOLD_RAD = math.radians(ANGLE_THRESHOLD_DEG)

def compute_angle(p_prev, p_center, p_next):
    """
    p_prev→p_center と p_center→p_next のベクトル間の角度を返す（ラジアン）
    p_* は (x, y) タプル
    """
    v1 = (p_prev[0] - p_center[0], p_prev[1] - p_center[1])
    v2 = (p_next[0] - p_center[0], p_next[1] - p_center[1])
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    n1 = math.hypot(*v1)
    n2 = math.hypot(*v2)
    if n1 == 0 or n2 == 0:
        return 0.0
    # 逆三角関数の引数を -1～1 にクランプ
    cosang = max(-1.0, min(1.0, dot / (n1 * n2)))
    return math.acos(cosang)

def main():
    infile  = 'log_anntei.csv'
    outfile = 'log.csv'

    try:
        with open(infile,  newline='') as fin, \
             open(outfile, 'w', newline='') as fout:

            reader = csv.reader(fin)
            writer = csv.writer(fout)

            # 最初の２行を読み込む（A, B）
            try:
                A_line = next(reader)
                B_line = next(reader)
            except StopIteration:
                print('入力ファイルの行数が足りません。', file=sys.stderr)
                return

            # 最初の点は必ず出力
            writer.writerow(A_line)
            A = (float(A_line[1]), float(A_line[2]))
            B = (float(B_line[1]), float(B_line[2]))

            # スライディングウィンドウで処理
            for C_line in reader:
                C = (float(C_line[1]), float(C_line[2]))
                
                # 直線AB と BC の角度
                angle_ABC = compute_angle(A, B, C)
                if angle_ABC <= ANGLE_THRESHOLD_RAD:
                    # 小さい角度なら B をスキップ
                    B_line = C_line
                    B = C
                    continue

                # 変化が大きい → 次の D を読んで二段階チェック
                try:
                    D_line = next(reader)
                except StopIteration:
                    # D がないときは最後の点を書いて終了
                    writer.writerow(B_line)
                    writer.writerow(C_line)
                    break

                D = (float(D_line[1]), float(D_line[2]))
                # 直線AC と CD の角度
                angle_ACD = compute_angle(A, C, D)
                if angle_ACD >= ANGLE_THRESHOLD_RAD:
                    # 十分に変化あり → A→C をキーとして出力
                    writer.writerow(C_line)
                    # ウィンドウを進める
                    A_line = C_line
                    A = C
                    B_line = D_line
                    B = D
                else:
                    # 変化小 → C をスキップ
                    B_line = D_line
                    B = D
                # 次ループへ

            # ループ抜け後、残りの末尾点を出力
            writer.writerow(B_line)

    except FileNotFoundError:
        print(f'ファイルが見つかりません: {infile}', file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f'エラー発生: {e}', file=sys.stderr)
        sys.exit(1)

if __name__ == '__main__':
    main()
