#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拆解 cnt=1 碎片写的来源:元数据(低扇区) vs 数据区(高扇区),并找反复写的扇区。
用法: python sdwr_cnt1_breakdown.py <log.txt>"""
import re, sys
from collections import Counter

lines = open(sys.argv[1], encoding='utf-8', errors='replace').read().splitlines()
rx = re.compile(r'\[SDwr\] dma=(\d+)ms prog=(\d+)ms cnt=(\d+) sec=(\d+)')
cnt1_secs = []
by_cnt = Counter()
for l in lines:
    m = rx.search(l)
    if not m:
        continue
    dma, prog, cnt, sec = int(m[1]), int(m[2]), int(m[3]), int(m[4])
    by_cnt[cnt] += 1
    if cnt == 1:
        cnt1_secs.append(sec)

lo = [s for s in cnt1_secs if s < 100000]
hi = [s for s in cnt1_secs if s >= 100000]
print("cnt=1 total:", len(cnt1_secs), " 低扇区(元数据/FAT):", len(lo), " 高扇区(数据):", len(hi))
print("低扇区值分布:", Counter(lo).most_common(10))
c = Counter(hi)
print("高扇区 cnt=1 最频繁扇区(次数):")
for s, n in c.most_common(15):
    print(f"  sec={s} x{n}")
# cnt 分布总览
print("cnt 分布:", dict(sorted(by_cnt.items())))
