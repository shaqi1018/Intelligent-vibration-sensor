#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""算 CSV 会话 vs BIN 会话的真实产出速率,和落盘上限对比,钉死主因。
用法: python csv_vs_bin_throughput.py"""
import os

def total_bytes(d, exts=('.CSV', '.BIN', '.WAV')):
    s = 0
    if not os.path.isdir(d):
        return None
    for f in os.listdir(d):
        p = os.path.join(d, f)
        if os.path.isfile(p) and f.upper().endswith(exts):
            s += os.path.getsize(p)
    return s

def report(name, d, dur_s):
    b = total_bytes(d)
    if b is None:
        print(f"[{name}] 目录不存在: {d}")
        return
    # 分数据(不含WAV)
    data = total_bytes(d, ('.CSV', '.BIN'))
    wav = b - data
    print(f"[{name}] 时长={dur_s}s")
    print(f"  总产出={b/1e6:.1f}MB  数据(非WAV)={data/1e6:.1f}MB  WAV={wav/1e6:.1f}MB")
    print(f"  总速率={b/dur_s/1024:.0f} KB/s  数据速率={data/dur_s/1024:.0f} KB/s  WAV速率={wav/dur_s/1024:.0f} KB/s")

# CSV 会话:duration_s=2107(从CONFIG.JSN timebase)
report("CSV 07-16-17-31", r"E:\CTBX_2026-07-16-17-31", 2107)
# BIN 会话:100.4分钟=6024s(记忆中最佳会话)
report("BIN 07-15-17-54", r"E:\CTBX_2026-07-15-17-54", 6024)
print()
print("落盘上限(实测本次CSV [SDwr]): 有效吞吐 ~148 KB/s (46.7MB/322.4s)")
print("→ 若数据速率 > 148KB/s 则环持续溢出=掉帧;BIN应<上限,CSV应>上限")
