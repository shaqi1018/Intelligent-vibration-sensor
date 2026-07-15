#!/usr/bin/env python3
"""导出一个通道所有 gap 的发生时刻(相对会话起点秒),用于跨通道对齐比对。
★2026-07-14 帧瘦身后无逐帧 ts:时刻/时长由 frame_id×interval_us 推算。
输出: 每行 'at_sec missing dur_ms'。用法: bin_gap_times.py <fsz> <interval_us> <seg...>"""
import sys, struct

fsz = int(sys.argv[1])
interval_us = float(sys.argv[2])
paths = sys.argv[3:]
data = b''.join(open(p, 'rb').read() for p in paths)
nf = len(data) // fsz

fids = []
for i in range(nf):
    fids.append(struct.unpack_from('<I', data, i*fsz)[0])

f0 = fids[0]
for fa, fb in zip(fids, fids[1:]):
    if fb > fa + 1:
        at = (fa - f0) * interval_us / 1e6
        miss = fb - fa - 1
        dur = miss * interval_us / 1e3
        print(f"{at:.2f}\t{miss}\t{dur:.1f}")
