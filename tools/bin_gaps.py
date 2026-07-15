#!/usr/bin/env python3
"""分析 BIN 帧 gap 的时间分布:每处 frame_id 跳变算出阻塞发生时刻、丢了多少帧、持续多久。
★2026-07-14 帧瘦身后无逐帧 timestamp:时间由 frame_id×interval_us 推算(合成 ts 本就这么来,
等价还原)。用法: bin_gaps.py <fsz> <interval_us> <seg...>
  fsz=14(LSM/H3/QMA 新格式) 或 22(MAG/旧格式)
  interval_us=每帧间隔微秒 = 1e6/ODR(如 LSM 6664Hz→150, H3 400Hz→2500, QMA 1600Hz→625)
帧新布局: frame_id(u32@0) data(3×int16@4) crc32(u32@10)"""
import sys, struct

fsz = int(sys.argv[1])
interval_us = float(sys.argv[2])
paths = sys.argv[3:]
data = b''.join(open(p, 'rb').read() for p in paths)
nf = len(data) // fsz

fids = []
for i in range(nf):
    off = i * fsz
    fids.append(struct.unpack_from('<I', data, off)[0])

f0 = fids[0]
def at_sec(fid):  # 相对会话起点秒:由 frame_id 推算
    return (fid - f0) * interval_us / 1e6

gaps = []
for fa, fb in zip(fids, fids[1:]):
    if fb > fa + 1:
        # start_fid,end_fid,missing,at_sec,dur_ms(缺帧数×间隔)
        gaps.append((fa, fb, fb-fa-1, at_sec(fa), (fb-fa-1)*interval_us/1e3))

print(f"total_frames={nf} total_gaps={len(gaps)} total_missing={sum(g[2] for g in gaps)}")
print(f"session_span_sec={at_sec(fids[-1]):.1f}")
print(f"\n最大的 15 处 gap (按丢帧数排序):")
print(f"{'at_sec':>9} {'missing':>8} {'blocked_ms':>10}  fid {'':>0}")
for g in sorted(gaps, key=lambda x:-x[2])[:15]:
    print(f"{g[3]:9.1f} {g[2]:8d} {g[4]:10.1f}  {g[0]}->{g[1]}")

# 时间分布:分 10 段看每段丢多少
print(f"\n丢帧的时间分布(10 段):")
span = at_sec(fids[-1])
for i in range(10):
    lo, hi = span*i/10, span*(i+1)/10
    m = sum(g[2] for g in gaps if lo <= g[3] < hi)
    c = sum(1 for g in gaps if lo <= g[3] < hi)
    print(f"  seg{i} [{lo:6.0f}-{hi:6.0f}s]: {c:4d} gaps, {m:8d} frames")
