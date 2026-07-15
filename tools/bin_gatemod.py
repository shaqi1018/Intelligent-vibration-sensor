#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检验坏点是否出现在每次写块边界。写块大小未知,故扫描一批候选块大小,
看哪个候选能让"坏点间距"最集中(即坏点≈k×blocksize)。"""
import sys, zlib, struct
from collections import Counter
path=sys.argv[1]; fsize=22
data=open(path,"rb").read(); N=len(data)
def ok(o):
    if o+fsize>N: return False
    fr=data[o:o+fsize]; return (zlib.crc32(fr[:-4])&0xFFFFFFFF)==struct.unpack("<I",fr[-4:])[0]
bad=[]; off=0
while off+fsize<=N:
    if ok(off): off+=fsize; continue
    bad.append(off)
    nxt=None
    for o in range(off+1,min(off+16384,N-fsize)):
        if ok(o) and ok(o+fsize): nxt=o; break
    if nxt is None: break
    off=nxt
print(f"坏点数={len(bad)}")
# 累积:假设每次真实写块写了 B 字节但只有 B-2 落盘(丢2),则第 k 块结尾的文件偏移
# = k*(B-2). 反推 B: 相邻坏点间距≈B-2. 已知间距众数:
gaps=[bad[i+1]-bad[i] for i in range(len(bad)-1)]
common=Counter(gaps).most_common(6)
print("间距众数:",common)
# 每个坏点错位方向(+2丢/其它):找每个坏点重对齐delta
deltas=Counter()
for o in bad[:2000]:
    for d in range(1,40):
        if ok(o+d) and ok(o+d+fsize):
            deltas[d]+=1; break
    else:
        deltas[-1]+=1
print("重对齐delta分布(前2000坏点):",deltas.most_common(8))
