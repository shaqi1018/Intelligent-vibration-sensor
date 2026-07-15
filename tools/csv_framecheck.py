#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查CSV首列 frame_id 是否严格连续(掉帧=跳号)。用法: python csv_framecheck.py <file>"""
import sys
path=sys.argv[1]
first=None; last=None; rows=0; jumps=0; jump_samples=[]
with open(path,"rb") as f:
    for ln in f:
        s=ln.decode("ascii","replace").strip()
        if s=="" or s.startswith("frame_id"): continue
        try: fid=int(s.split(",")[0])
        except: continue
        rows+=1
        if first is None: first=fid
        elif fid!=last+1:
            jumps+=1
            if len(jump_samples)<5: jump_samples.append((last,fid,fid-last-1))
        last=fid
gap_total = (last-first+1-rows) if first is not None else 0
print(f"{path.split('/')[-1]}: 行数={rows} fid[{first}..{last}] 期望={last-first+1 if first else 0} 跳号次数={jumps} 累计丢={gap_total}")
for a,b,n in jump_samples:
    print(f"   跳号: {a}->{b} (丢{n})")
