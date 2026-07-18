#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""看丢帧是否随时间均匀 vs 突发聚集。把fid空间切成N桶,报每桶丢帧率。
用法: python csv_gaptime.py <nfields> <seg1> [seg2 ...]"""
import sys
nfields = int(sys.argv[1])
segs = sys.argv[2:]

fids = []
prev = None
for path in segs:
    with open(path, "r", encoding="ascii", errors="replace") as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("frame_id") or s.startswith("#"):
                continue
            parts = s.split(",")
            if len(parts) != nfields:
                continue
            try:
                fid = int(parts[0])
            except ValueError:
                continue
            if prev is not None and fid <= prev:
                continue  # skip nonmono garbage
            fids.append(fid)
            prev = fid

fmin, fmax = fids[0], fids[-1]
span = fmax - fmin + 1
N = 20
buckets_present = [0]*N
for fid in fids:
    b = (fid - fmin) * N // span
    if b >= N: b = N-1
    buckets_present[b] += 1
per = span / N
print(f"fid[{fmin}..{fmax}] span={span:,} rows={len(fids):,}  每桶期望={per:,.0f}")
for i in range(N):
    got = buckets_present[i]
    drop = (1 - got/per)*100
    bar = "#" * int(max(0,drop)/2)
    print(f"  桶{i:2d} 得{got:8,} 丢{drop:5.1f}% {bar}")
