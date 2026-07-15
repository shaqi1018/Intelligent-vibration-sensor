#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看前若干坏点的 (文件偏移, 重对齐delta, 该块字节数相对gate) 序列,
判断 +2/-2 是否成对(净零错位=数据没丢只挪位) 还是单向累积(真丢字节)。"""
import sys, zlib, struct
path=sys.argv[1]; gate=int(sys.argv[2]); fsize=22
data=open(path,"rb").read(); N=len(data)
def ok(o):
    if o+fsize>N: return False
    fr=data[o:o+fsize]; return (zlib.crc32(fr[:-4])&0xFFFFFFFF)==struct.unpack("<I",fr[-4:])[0]
off=0; rows=[]; prev=0
while off+fsize<=N and len(rows)<30:
    if ok(off): off+=fsize; continue
    # 找重对齐
    nxt=None
    for d in range(1,64):
        if ok(off+d) and ok(off+d+fsize): nxt=off+d; break
    delta = (nxt-off) if nxt else None
    rows.append((off, off-prev, delta))
    prev=off
    if nxt is None: break
    off=nxt
print("(坏点偏移, 距上一坏点, 重对齐delta):")
cum=0
for o,g,d in rows:
    net = (d - ((d//fsize)*fsize)) if d else 0   # d对22取余=净错位
    print(f"  off={o} gap={g} delta={d} 余={d%fsize if d else '-'}")
