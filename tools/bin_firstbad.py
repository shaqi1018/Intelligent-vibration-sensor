#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位第一个 CRC 坏帧,dump 其前后原始字节,判断是"字节被改"还是"字节丢失导致错位"。
若在坏帧附近按 frame_id 递增重新对齐能找回,则是"丢字节/错位"而非"损坏"。
用法: python bin_firstbad.py <file.bin> <frame_size>
"""
import sys, zlib, struct

path = sys.argv[1]
fsize = int(sys.argv[2])
data = open(path, "rb").read()
n = len(data) // fsize

def crc_ok(off):
    fr = data[off:off+fsize]
    return (zlib.crc32(fr[:-4]) & 0xFFFFFFFF) == struct.unpack("<I", fr[-4:])[0]

def fid(off):
    return struct.unpack("<I", data[off:off+4])[0]

# 找第一个坏帧
first_bad = None
for i in range(n):
    if not crc_ok(i*fsize):
        first_bad = i
        break

if first_bad is None:
    print("no bad frame")
    sys.exit(0)

off = first_bad * fsize
print(f"first_bad frame_index={first_bad} byte_offset={off} (0x{off:X})")
print(f"prev frame fid={fid(off-fsize)} crc_ok={crc_ok(off-fsize)}")
print(f"bad  frame fid={fid(off)} crc_ok={crc_ok(off)}")
# dump 前一帧 + 坏帧 + 后两帧 的原始字节
dump = data[off-fsize: off+3*fsize]
print("raw around (prev|bad|+1|+2):")
for k in range(0, len(dump), fsize):
    print(f"  +{k-fsize:+4d}: {dump[k:k+fsize].hex()}")

# 尝试:从坏帧起,在 ±40 字节窗口内滑动,看能否找到一个偏移使得后续 8 帧 CRC 全对
prev = fid(off-fsize)
print(f"\n滑动重对齐搜索(prev_fid={prev},期望 bad_fid={prev+1}):")
for shift in range(-8, 41):
    test = off + shift
    if test < 0: continue
    if crc_ok(test) and fid(test) == (prev+1) & 0xFFFFFFFF:
        # 验证连续 8 帧
        good = all(crc_ok(test+j*fsize) for j in range(8))
        print(f"  shift={shift:+d}: bad帧在此偏移 CRC_OK 且 fid连续, 后8帧全对={good}")
        if good:
            print(f"  => 结论:此处丢/多了 {shift} 字节导致错位,数据本身未损坏")
            break
else:
    print("  窗口内未找到干净重对齐点(可能是真字节损坏,非错位)")
