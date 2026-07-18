#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按扇区区间给 cnt=1 分类,并看 cnt=1 与 cnt=127 的时间交织模式。
用法: python sdwr_timeline.py <log.txt>"""
import re, sys
from collections import Counter

lines = open(sys.argv[1], encoding='utf-8', errors='replace').read().splitlines()
rx = re.compile(r'\[SDwr\] dma=(\d+)ms prog=(\d+)ms cnt=(\d+) sec=(\d+)')
seq = []
for l in lines:
    m = rx.search(l)
    if m:
        seq.append((int(m[1]), int(m[3]), int(m[4])))

# 高扇区 cnt=1 的扇区末3位分布(判断是不是文件尾扇区回写:随机 vs 固定簇偏移)
hi1 = [sec for dma, cnt, sec in seq if cnt == 1 and sec >= 100000]
# 相邻扇区差:cnt=1 紧跟 cnt=127 时,sec 是否= 前一个127块的尾+1
print("=== cnt=1(高扇区) 与前一条的关系 ===")
follow127_contig = 0   # cnt1 紧跟 cnt127 且扇区连续(=尾扇区续写)
follow127_jump = 0     # cnt1 紧跟 cnt127 但扇区跳变(=别的文件/FAT)
for i in range(1, len(seq)):
    dma, cnt, sec = seq[i]
    if cnt == 1 and sec >= 100000:
        pdma, pcnt, psec = seq[i-1]
        if pcnt == 127:
            if psec < sec <= psec + 130:
                follow127_contig += 1
            else:
                follow127_jump += 1
print(f"  cnt=1 紧跟 cnt=127 且扇区连续(尾扇区续写): {follow127_contig}")
print(f"  cnt=1 紧跟 cnt=127 但扇区跳变(切文件/FAT): {follow127_jump}")

# cnt=1 扇区值模 128(64KB簇=128扇区)看是否落在簇边界
mod128 = Counter(sec % 128 for sec in hi1)
print("=== 高扇区 cnt=1 的 (sec mod 128) top10 (若集中=簇内固定偏移) ===")
for k, v in mod128.most_common(10):
    print(f"  mod128={k}: {v}")

# 时间维度:总写次数、总 dma、若把所有 cnt<127 的写合并能省多少
total = len(seq)
small = [(dma, cnt, sec) for dma, cnt, sec in seq if cnt < 127]
big = [(dma, cnt, sec) for dma, cnt, sec in seq if cnt == 127]
sdma = sum(d for d, c, s in small)
bdma = sum(d for d, c, s in big)
print(f"\n=== 写次数/耗时 ===")
print(f"  总写: {total}  小块(cnt<127): {len(small)} 耗时 {sdma/1000:.1f}s  满块(127): {len(big)} 耗时 {bdma/1000:.1f}s")
print(f"  小块平均 {sdma/max(1,len(small)):.0f}ms/次  满块平均 {bdma/max(1,len(big)):.0f}ms/次")
