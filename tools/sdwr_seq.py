#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析 [SDwr] 写序列:cnt=1 前后紧邻的是什么写,判断 cnt=1 是否为大块写的尾扇区回写。
用法: python sdwr_seq.py <log.txt>"""
import re, sys
from collections import Counter

lines = open(sys.argv[1], encoding='utf-8', errors='replace').read().splitlines()
rx = re.compile(r'\[SDwr\] dma=(\d+)ms prog=(\d+)ms cnt=(\d+) sec=(\d+)')
seq = []
for l in lines:
    m = rx.search(l)
    if m:
        seq.append((int(m[1]), int(m[3]), int(m[4])))  # dma, cnt, sec

# 对每个 cnt=1,看它前一条的 cnt
prev_of_cnt1 = Counter()
pair_5680 = 0
for i in range(1, len(seq)):
    dma, cnt, sec = seq[i]
    if cnt == 1:
        pcnt = seq[i-1][1]
        prev_of_cnt1[pcnt] += 1

print("cnt=1 的前一条写的 cnt 分布(前值->出现次数):")
for k, v in prev_of_cnt1.most_common():
    print(f"  prev_cnt={k} -> {v}")

# 连续 cnt=1 的游程长度分布
runs = []
cur = 0
for dma, cnt, sec in seq:
    if cnt == 1:
        cur += 1
    else:
        if cur:
            runs.append(cur)
        cur = 0
if cur:
    runs.append(cur)
print("连续 cnt=1 游程长度分布:", Counter(runs).most_common(12))
print("cnt=1 游程总数:", len(runs), " 最长游程:", max(runs) if runs else 0)
