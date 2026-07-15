#!/usr/bin/env python3
"""BIN 帧审计:定长帧 frame_id(offset0,u32 LE) + data + crc32(末4B)。
★2026-07-14 帧瘦身:LSM/H3/QMA 新格式 14B(fid4 + 3×int16 + crc4,无 ts);MAG/旧数据仍 22B。
本工具只数 frame_id 连续性+校验末4B CRC,与中间布局无关 → 传对 frame_size 即可。
用法: bin_audit.py <file> [frame_size=14]"""
import sys, struct, zlib

def audit(path, fsz=22):
    with open(path, 'rb') as f:
        data = f.read()
    n_bytes = len(data)
    n_frames = n_bytes // fsz
    tail = n_bytes % fsz
    fids = []
    bad_crc = 0
    crc_checked = 0
    for i in range(n_frames):
        off = i * fsz
        frame = data[off:off+fsz]
        fid = struct.unpack_from('<I', frame, 0)[0]
        fids.append(fid)
        # CRC32 over frame minus last 4 bytes
        stored = struct.unpack_from('<I', frame, fsz-4)[0]
        calc = zlib.crc32(frame[:fsz-4]) & 0xFFFFFFFF
        crc_checked += 1
        if calc != stored:
            bad_crc += 1
    if not fids:
        print(f"{path}: EMPTY"); return
    fmin, fmax = min(fids), max(fids)
    uniq = len(set(fids))
    span = fmax - fmin + 1
    # 连续性:按帧顺序检查 fid 是否 +1 递增
    mono = sum(1 for a,b in zip(fids, fids[1:]) if b == a+1)
    gaps = sum(1 for a,b in zip(fids, fids[1:]) if b > a+1)
    gap_missing = sum((b-a-1) for a,b in zip(fids, fids[1:]) if b > a+1)
    backward = sum(1 for a,b in zip(fids, fids[1:]) if b <= a)
    print(f"=== {path} ===")
    print(f"  bytes={n_bytes} frame_size={fsz} n_frames={n_frames} tail_bytes={tail}")
    print(f"  frame_id: min={fmin} max={fmax} unique={uniq} span={span}")
    print(f"  written={n_frames}  span={span}  missing_vs_span={span-uniq} ({100.0*(span-uniq)/span:.2f}%)")
    print(f"  monotone_+1={mono}  forward_gaps={gaps} (missing {gap_missing})  backward/dup={backward}")
    print(f"  CRC: checked={crc_checked} bad={bad_crc} ({100.0*bad_crc/max(1,crc_checked):.3f}%)")

if __name__ == '__main__':
    path = sys.argv[1]
    fsz = int(sys.argv[2]) if len(sys.argv) > 2 else 14
    audit(path, fsz)
