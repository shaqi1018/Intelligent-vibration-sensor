#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Analyze [SDwr] dma/prog vs cnt from serial log to decide double-buffer viability."""
import re, sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\Q1\Desktop\新建文本文档.txt"
txt = open(path, encoding="utf-8", errors="replace").read()

rows = []
for m in re.finditer(r"\[SDwr\] dma=(\d+)ms prog=(\d+)ms cnt=(\d+) sec=(\d+)", txt):
    dma, prog, cnt, sec = map(int, m.groups())
    rows.append((dma, prog, cnt, sec))

print(f"total [SDwr] samples: {len(rows)}")

# 1) dma time vs cnt bucket — is dma independent of size?
buckets = defaultdict(list)
for dma, prog, cnt, sec in rows:
    if cnt == 1: b = "cnt=1 (512B/FAT)"
    elif cnt <= 8: b = "cnt=2..8"
    elif cnt <= 32: b = "cnt=9..32"
    elif cnt <= 64: b = "cnt=33..64"
    elif cnt <= 96: b = "cnt=65..96"
    else: b = "cnt=97..127"
    buckets[b].append(dma)

order = ["cnt=1 (512B/FAT)","cnt=2..8","cnt=9..32","cnt=33..64","cnt=65..96","cnt=97..127"]
print("\n=== dma(ms) by block size — if flat, size doesn't matter (fixed per-write cost) ===")
print(f"{'bucket':<18}{'n':>6}{'mean':>8}{'min':>6}{'max':>6}{'KB/write':>10}{'MB/s':>8}")
for b in order:
    v = buckets[b]
    if not v: continue
    mean = sum(v)/len(v)
    # approx avg cnt in bucket midpoint for MB/s
    print(f"{b:<18}{len(v):>6}{mean:>8.0f}{min(v):>6}{max(v):>6}")

# 2) cnt=1 (single-sector) share and total time wasted
cnt1 = [r for r in rows if r[2]==1]
cnt1_dma = sum(r[0] for r in cnt1)
all_dma = sum(r[0] for r in rows)
print(f"\n=== single-sector (cnt=1) writes ===")
print(f"count: {len(cnt1)} / {len(rows)}  = {100*len(cnt1)/len(rows):.1f}% of all writes")
print(f"dma time in cnt=1: {cnt1_dma} ms / {all_dma} ms total = {100*cnt1_dma/all_dma:.1f}% of all write-wait time")
print(f"bytes moved by cnt=1: {len(cnt1)*512} B = {len(cnt1)*512/1024:.0f} KB (for {cnt1_dma} ms!)")

# 3) low-sector (FAT/dir) single-sector writes
low = [r for r in cnt1 if r[3] < 100000]
print(f"cnt=1 at low sector (<100k, FAT/dir/FSINFO): {len(low)}  → {sum(r[0] for r in low)} ms")

# 4) effective throughput: total bytes / total dma time
tot_bytes = sum(r[2]*512 for r in rows)
print(f"\n=== effective write throughput (data through SDMMC) ===")
print(f"total bytes: {tot_bytes/1024/1024:.1f} MB, total dma wait: {all_dma/1000:.1f} s")
print(f"aggregate: {tot_bytes/1024/(all_dma/1000):.0f} KB/s  (this is the wall we hit)")

# 5) what if every cnt=1 FAT write were eliminated (bigger cluster)?
non1_bytes = sum(r[2]*512 for r in rows if r[2]>1)
non1_dma = sum(r[0] for r in rows if r[2]>1)
print(f"\n=== hypothetical: remove all cnt=1 writes (perfect cluster alignment) ===")
print(f"data writes only: {non1_bytes/1024/1024:.1f} MB in {non1_dma/1000:.1f} s = {non1_bytes/1024/(non1_dma/1000):.0f} KB/s")
print(f"time saved: {cnt1_dma/1000:.1f} s ({100*cnt1_dma/all_dma:.0f}% of write budget)")
