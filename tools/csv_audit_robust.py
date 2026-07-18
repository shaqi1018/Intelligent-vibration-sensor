#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抗畸形行CSV审计。用法: python csv_audit_robust.py <name> <nfields> <seg1> [seg2 ...]
分开统计:总数据行/干净行(字段数对且fid顺序合理)/畸形行(字节损坏)/真丢帧(干净行间小正跳)。
用 MAX_JUMP 拒绝被粘连污染的fid(视为畸形,不计入丢帧)。"""
import sys

name = sys.argv[1]
nfields = int(sys.argv[2])
segs = sys.argv[3:]
MAX_JUMP = 100000  # 合理正跳上限;超过视为粘连污染的fid

total = 0          # 所有非空非表头行
clean = 0          # 字段数正确的行
badfields = 0      # 字段数错误(字节损坏铁证)
last = None
first_clean = None
last_clean = None
true_drop = 0      # 干净行之间的小正跳累计
n_gaps = 0
bad_samples = []
gap_samples = []

for si, path in enumerate(segs):
    with open(path, "rb") as f:
        for raw in f:
            s = raw.decode("ascii", "replace").strip()
            if s == "" or s.startswith("frame_id") or s.startswith("#"):
                continue
            total += 1
            parts = s.split(",")
            ok_fields = (len(parts) == nfields)
            if not ok_fields:
                badfields += 1
                if len(bad_samples) < 6:
                    bad_samples.append((si, s[:48]))
                continue
            try:
                fid = int(parts[0])
            except Exception:
                badfields += 1
                continue
            # fid合理性:相对上一个干净fid的跳变在 [1, MAX_JUMP]
            if last_clean is not None:
                d = fid - last_clean
                if d <= 0 or d > MAX_JUMP:
                    # fid本身被污染(粘连) -> 归为畸形,不更新序列锚
                    badfields += 1
                    if len(bad_samples) < 6:
                        bad_samples.append((si, "FIDBAD:" + s[:40]))
                    continue
                if d > 1:
                    true_drop += (d - 1)
                    n_gaps += 1
                    if len(gap_samples) < 8:
                        gap_samples.append((last_clean, fid, d - 1))
            clean += 1
            if first_clean is None:
                first_clean = fid
            last_clean = fid

span = (last_clean - first_clean + 1) if first_clean is not None else 0
# 真丢帧率 = 干净序列里的缺口 / 序列跨度
true_drop_pct = (true_drop / span * 100.0) if span else 0.0
bad_pct = (badfields / total * 100.0) if total else 0.0
print(f"[{name}] segs={len(segs)} 总行={total:,} 干净={clean:,} 畸形={badfields:,} ({bad_pct:.3f}%)")
print(f"    fid跨度[{first_clean}..{last_clean}]={span:,}  真丢帧(干净序列缺口)={true_drop:,} 缺口数={n_gaps} 丢帧率={true_drop_pct:.3f}%")
for a, b, n in gap_samples:
    print(f"    缺口: {a}->{b} (丢{n})")
for si, txt in bad_samples:
    print(f"    畸形(seg{si}): {txt}")
