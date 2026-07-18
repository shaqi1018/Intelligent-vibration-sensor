#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""稳健跨段CSV审计: 假定fid单调递增,忽略末尾/畸形行污染。
用法: python csv_audit2.py <name> <nfields> <seg1> [seg2 ...]"""
import sys

name = sys.argv[1]
nfields = int(sys.argv[2])
segs = sys.argv[3:]

fmin = None
fmax = None
valid = 0            # 字段数正确且fid可解析
badfields = 0        # 字段数不对
nonmono = 0          # fid <= prev (回退/重复)
prev = None
nonmono_samples = []

for si, path in enumerate(segs):
    with open(path, "r", encoding="ascii", errors="replace") as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith("frame_id") or s.startswith("#"):
                continue
            parts = s.split(",")
            if len(parts) != nfields:
                badfields += 1
                continue
            try:
                fid = int(parts[0])
            except ValueError:
                badfields += 1
                continue
            valid += 1
            if fmin is None or fid < fmin:
                fmin = fid
            if fmax is None or fid > fmax:
                fmax = fid
            if prev is not None and fid <= prev:
                nonmono += 1
                if len(nonmono_samples) < 6:
                    nonmono_samples.append((prev, fid))
            prev = fid

span = (fmax - fmin + 1) if fmin is not None else 0
missing = span - valid
drop_pct = (missing / span * 100.0) if span else 0.0
print(f"[{name}] segs={len(segs)} valid_rows={valid:,} fid[{fmin}..{fmax}] span={span:,}")
print(f"    missing={missing:,}  drop={drop_pct:.3f}%  badfields={badfields}  nonmono={nonmono}")
for a, b in nonmono_samples:
    print(f"    nonmono: {a} -> {b}")
