#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv

INPUT_FILE = 'log_anntei.csv'
OUTPUT_FILE = 'log.csv'
SAMPLE_RATE = 10  # 10行に1行ずつ

def downsample_csv(input_path, output_path, rate):
    with open(input_path, newline='') as infile, \
         open(output_path, 'w', newline='') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)

        for idx, row in enumerate(reader):
            # idx=0,10,20,... の行を出力
            if idx % rate == 0:
                writer.writerow(row)

if __name__ == '__main__':
    downsample_csv(INPUT_FILE, OUTPUT_FILE, SAMPLE_RATE)
    print(f"{INPUT_FILE} から {SAMPLE_RATE}行に1行ずつ抽出して {OUTPUT_FILE} を作成しました。")
