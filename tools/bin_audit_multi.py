#!/usr/bin/env python3
"""多段 BIN 帧审计:把同一通道的多个分段文件按顺序拼起来数 frame_id 连续性。
★2026-07-14 帧瘦身:LSM/H3/QMA 新格式 frame_size=14(无 ts);MAG/旧数据=22。
用法: bin_audit_multi.py <frame_size> <seg1.BIN> <seg2.BIN> ..."""
import sys, struct, zlib

fsz = int(sys.argv[1])
paths = sys.argv[2:]
fids = []
bad_crc = 0
tail_total = 0
# ★段边界会把一帧切成两半(001 末尾半帧 + 002 开头半帧),必须先拼成连续字节流再切帧。
data = b''.join(open(p, 'rb').read() for p in paths)
total_frames = len(data) // fsz
tail_total = len(data) % fsz
for i in range(total_frames):
    off = i * fsz
    frame = data[off:off+fsz]
    fids.append(struct.unpack_from('<I', frame, 0)[0])
    stored = struct.unpack_from('<I', frame, fsz-4)[0]
    if (zlib.crc32(frame[:fsz-4]) & 0xFFFFFFFF) != stored:
        bad_crc += 1

fmin, fmax = min(fids), max(fids)
uniq = len(set(fids))
span = fmax - fmin + 1
gaps = sum(1 for a,b in zip(fids, fids[1:]) if b > a+1)
gap_missing = sum((b-a-1) for a,b in zip(fids, fids[1:]) if b > a+1)
backward = sum(1 for a,b in zip(fids, fids[1:]) if b <= a)
print(f"=== {len(paths)} segs: {', '.join(paths)} ===")
print(f"  total_frames={total_frames} tail_bytes={tail_total}")
print(f"  frame_id: min={fmin} max={fmax} unique={uniq} span={span}")
print(f"  真实丢帧 missing_vs_span={span-uniq} ({100.0*(span-uniq)/span:.2f}%)")
print(f"  forward_gaps={gaps} (missing {gap_missing})  backward/dup={backward}")
print(f"  CRC: bad={bad_crc} ({100.0*bad_crc/max(1,total_frames):.3f}%)")
