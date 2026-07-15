#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""收集所有错位/坏点的绝对字节偏移,分析它们相对 512B 扇区 / 32KB 簇 / 64KB FLUSH_CHUNK
边界的对齐分布。若坏点集中在这些边界 => 写/DMA 边界问题;若随机 => 别的。"""
import sys, zlib, struct
path=sys.argv[1]; fsize=int(sys.argv[2])
data=open(path,"rb").read(); N=len(data)
def ok(o):
    if o+fsize>N: return False
    fr=data[o:o+fsize]; return (zlib.crc32(fr[:-4])&0xFFFFFFFF)==struct.unpack("<I",fr[-4:])[0]

# 逐字节游走,记录每个"错位段"的起始绝对偏移
bad_offsets=[]
off=0
while off+fsize<=N:
    if ok(off): off+=fsize; continue
    bad_offsets.append(off)
    nxt=None
    for o in range(off+1, min(off+8192,N-fsize)):
        if ok(o) and ok(o+fsize): nxt=o; break
    if nxt is None: break
    off=nxt

print(f"总错位段={len(bad_offsets)}")
from collections import Counter
for boundary,name in [(512,"扇区512"),(32768,"簇32K"),(65536,"FLUSH64K")]:
    mods=Counter(o % boundary for o in bad_offsets)
    top=mods.most_common(5)
    at0=mods.get(0,0)
    print(f"  相对{name}: 落在边界(mod==0)的={at0}/{len(bad_offsets)}  top5余数={top}")
# 坏点间距分布
if len(bad_offsets)>1:
    gaps=[bad_offsets[i+1]-bad_offsets[i] for i in range(len(bad_offsets)-1)]
    gc=Counter(gaps).most_common(8)
    print(f"  坏点间距top8: {gc}")
