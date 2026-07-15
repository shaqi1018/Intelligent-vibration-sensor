#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位每个坏行的字节偏移,分析相对512扇区边界的分布。用法: python csv_badpos.py <file> <fields>"""
import sys
from collections import Counter
path=sys.argv[1]; expect=int(sys.argv[2])
data=open(path,"rb").read()
def line_ok(ln):
    parts=ln.split(b",")
    if len(parts)!=expect: return False
    for p in parts:
        s=p.strip()
        if s==b"" or any(c not in b"0123456789.-" for c in s): return False
    return True
off=0; bad_offsets=[]
for ln in data.split(b"\n"):
    L=len(ln)+1  # +\n
    lr=ln.rstrip(b"\r")
    if lr!=b"" and not lr.startswith(b"frame_id"):
        if not line_ok(lr):
            bad_offsets.append(off)
    off+=L
print(f"坏行数={len(bad_offsets)}")
if bad_offsets:
    mods=Counter(o % 512 for o in bad_offsets)
    at_near_boundary=sum(v for k,v in mods.items() if k>=500 or k<=12)
    print(f"  相对512: 落在边界±12的={at_near_boundary}/{len(bad_offsets)}  top8余数={mods.most_common(8)}")
    # 间距
    gaps=[bad_offsets[i+1]-bad_offsets[i] for i in range(min(len(bad_offsets)-1,5000))]
    gc=Counter(gaps).most_common(6)
    print(f"  坏行间距top6: {gc}")
