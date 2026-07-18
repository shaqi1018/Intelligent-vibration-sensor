#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""跨段CSV审计: frame_id连续性(丢帧)+ 字段数畸形行检查。
用法: python csv_audit_multi.py <name> <expected_fields> <seg1> [seg2 ...]
跨段累计fid; 段边界1行畸形视为切割边界效应(单独计数)。"""
import sys

name = sys.argv[1]
nfields = int(sys.argv[2])
segs = sys.argv[3:]

first = None
last = None
rows = 0
jumps = 0
gap_total = 0
badfields = 0
jump_samples = []
bad_samples = []

for si, path in enumerate(segs):
    with open(path, "rb") as f:
        for raw in f:
            s = raw.decode("ascii", "replace").strip()
            if s == "" or s.startswith("frame_id") or s.startswith("#"):
                continue
            parts = s.split(",")
            # field count check
            if len(parts) != nfields:
                badfields += 1
                if len(bad_samples) < 5:
                    bad_samples.append((si, s[:40]))
                # 仍尝试取fid
            try:
                fid = int(parts[0])
            except Exception:
                continue
            rows += 1
            if first is None:
                first = fid
            elif fid != last + 1:
                if fid > last:
                    gap_total += (fid - last - 1)
                jumps += 1
                if len(jump_samples) < 8:
                    jump_samples.append((last, fid, fid - last - 1))
            last = fid

expected = (last - first + 1) if first is not None else 0
drop_pct = (gap_total / expected * 100.0) if expected else 0.0
print(f"[{name}] segs={len(segs)} rows={rows:,} fid[{first}..{last}] 期望连续={expected:,}")
print(f"    跳号次数={jumps} 累计丢帧={gap_total:,} 丢帧率={drop_pct:.3f}%  字段畸形行={badfields}")
for a, b, n in jump_samples:
    print(f"    跳号: {a}->{b} (丢{n})")
for si, txt in bad_samples:
    print(f"    畸形(seg{si}): {txt}")
