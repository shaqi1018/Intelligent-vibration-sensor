#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""探测真实簇大小:cnt=1 高扇区对不同簇尺寸取模,看哪个尺寸下 cnt=1 集中在单一偏移。
若簇=S 扇区时 (sec mod S) 高度集中 → 真实簇=S,且 cnt=1 落在簇边界=写块跨簇。
用法: python sdwr_cluster_probe.py <log.txt>"""
import re, sys
from collections import Counter

lines = open(sys.argv[1], encoding='utf-8', errors='replace').read().splitlines()
rx = re.compile(r'\[SDwr\] dma=(\d+)ms prog=(\d+)ms cnt=(\d+) sec=(\d+)')
hi1 = []
allbig = []  # cnt=127 起始扇区
for l in lines:
    m = rx.search(l)
    if not m:
        continue
    dma, prog, cnt, sec = int(m[1]), int(m[2]), int(m[3]), int(m[4])
    if sec >= 100000:
        if cnt == 1:
            hi1.append(sec)
        if cnt == 127:
            allbig.append(sec)

print(f"高扇区 cnt=1 总数: {len(hi1)}  cnt=127 总数: {len(allbig)}")
for S in [32, 64, 128, 256]:  # 16KB,32KB,64KB,128KB 簇
    c = Counter(sec % S for sec in hi1)
    top = c.most_common(3)
    concentration = top[0][1] / len(hi1) * 100 if hi1 else 0
    print(f"簇={S}扇区({S*512//1024}KB): cnt=1 mod{S} top3={top}  最高偏移占比={concentration:.0f}%")

# cnt=127 起始扇区对簇取模:若都对齐则应集中在 0 或固定值
print("\n=== cnt=127(满块)起始扇区对齐检查 ===")
for S in [128]:
    c = Counter(sec % S for sec in allbig)
    print(f"簇={S}: cnt=127 起始 mod{S} top5={c.most_common(5)}")
