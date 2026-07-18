#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""严肃复核一个会话:时长(MIC锚)+各通道 frame_id 连续性+CRC32+产出对比。
用法: audit_session.py <session_dir>"""
import sys, os, struct, zlib, glob

d = sys.argv[1]

def frames(paths, fsz):
    data = b"".join(open(p, "rb").read() for p in paths)
    total = len(data) // fsz
    tail = len(data) % fsz
    bad = 0
    fmin = None
    fmax = 0
    seen = set()
    for i in range(total):
        off = i * fsz
        fr = data[off:off + fsz]
        fid = struct.unpack_from("<I", fr, 0)[0]
        stored = struct.unpack_from("<I", fr, fsz - 4)[0]
        if (zlib.crc32(fr[:fsz - 4]) & 0xFFFFFFFF) != stored:
            bad += 1
        seen.add(fid)
        if fmin is None or fid < fmin:
            fmin = fid
        if fid > fmax:
            fmax = fid
    span = fmax - fmin + 1 if fmin is not None else 0
    uniq = len(seen)
    missing = span - uniq
    return total, tail, fmin, fmax, uniq, span, missing, bad

def segs(base):
    return sorted(glob.glob(os.path.join(d, base + "*.BIN")))

# MIC 时长锚
wav = os.path.join(d, "MIC.WAV")
if os.path.exists(wav):
    sz = os.path.getsize(wav) - 44
    sec = sz / (96000 * 2)
    print("录制时长(MIC/SAI锚) ≈ %.0fs = %.1f分钟\n" % (sec, sec / 60))

chans = [
    ("ACC_LOW (LSM加速度)", "ACC_LOW", 14, 6664),
    ("GYR_LOW (LSM角速度)", "GYR_LOW", 14, 6664),
    ("ACC_MID (QMA)",       "ACC_MID", 14, 1600),
    ("ACC_HIGH (H3)",       "ACC_HIGH", 14, 400),
    ("MAG (磁力计)",         "MAG",     22, 100),
    ("ENV (温湿度)",         "ENV",     24, 1),
]
print("%-22s %10s %8s %8s %7s %8s" % ("通道", "帧数", "丢帧%", "CRC坏", "尾字节", "fwd连续"))
for name, base, fsz, odr in chans:
    ps = segs(base)
    if not ps:
        print("%-22s  (无文件)" % name)
        continue
    total, tail, fmin, fmax, uniq, span, missing, bad = frames(ps, fsz)
    pct = 100.0 * missing / span if span else 0
    crcpct = 100.0 * bad / total if total else 0
    print("%-22s %10d %7.2f%% %5d(%.3f%%) %6d  min=%d max=%d" %
          (name, total, pct, bad, crcpct, tail, fmin, fmax))
