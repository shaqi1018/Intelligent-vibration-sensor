#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""逐帧校验 BIN 文件 CRC32,统计坏帧率并抓样本。
固件 CRC32 = 标准 zlib(init 0xFFFFFFFF,反射,末异或)== zlib.crc32。
每帧末 4 字节为 crc32(小端),覆盖帧内除 crc32 外的全部字节。
用法: python verify_bin_crc.py <file.bin> <frame_size>
"""
import sys, zlib, struct

def verify(path, fsize):
    total = 0
    bad = 0
    bad_ids = []
    prev_fid = None
    fid_jumps = 0
    with open(path, "rb") as f:
        while True:
            frame = f.read(fsize)
            if len(frame) < fsize:
                break
            total += 1
            payload = frame[:-4]
            stored = struct.unpack("<I", frame[-4:])[0]
            calc = zlib.crc32(payload) & 0xFFFFFFFF
            fid = struct.unpack("<I", frame[:4])[0]
            if prev_fid is not None and fid != (prev_fid + 1) & 0xFFFFFFFF:
                fid_jumps += 1
            prev_fid = fid
            if calc != stored:
                bad += 1
                if len(bad_ids) < 20:
                    bad_ids.append((total, fid, stored, calc, frame.hex()))
    return total, bad, bad_ids, fid_jumps

if __name__ == "__main__":
    path = sys.argv[1]
    fsize = int(sys.argv[2])
    total, bad, samples, jumps = verify(path, fsize)
    pct = (100.0 * bad / total) if total else 0.0
    print(f"file={path}")
    print(f"total_frames={total} bad_crc={bad} bad_pct={pct:.4f}% frame_id_jumps={jumps}")
    for n, fid, stored, calc, hx in samples:
        print(f"  #{n} fid={fid} stored={stored:08X} calc={calc:08X} raw={hx}")
