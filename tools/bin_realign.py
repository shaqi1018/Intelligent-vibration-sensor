#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从第一个坏帧字节偏移起,在其后一段字节流里暴力找"下一个能连续通过CRC的对齐点",
得出错位了多少字节(正=多出/插入,负=丢失)。用绝对字节偏移扫描,不假设22对齐。
用法: python bin_realign.py <file.bin> <frame_size>
"""
import sys, zlib, struct
path=sys.argv[1]; fsize=int(sys.argv[2])
data=open(path,"rb").read()
N=len(data)

def crc_ok_at(off):
    if off+fsize>N: return False
    fr=data[off:off+fsize]
    return (zlib.crc32(fr[:-4])&0xFFFFFFFF)==struct.unpack("<I",fr[-4:])[0]

# 找第一个坏帧(按22对齐)
first=None
for i in range(N//fsize):
    if not crc_ok_at(i*fsize):
        first=i*fsize; break
print(f"first_bad_off={first} (frame#{first//fsize})")

# 从 first 起,逐字节找一个 off 使得连续 6 帧 CRC 全过
found=None
for off in range(first, min(first+4096, N-6*fsize)):
    if all(crc_ok_at(off+j*fsize) for j in range(6)):
        found=off; break
if found is None:
    print("4KB窗口内找不到重对齐点")
else:
    delta=found-first
    print(f"realign_off={found}  delta={delta} 字节 (相对22对齐: {delta%fsize})")
    print(f"  => 从坏点到重对齐点,字节流{'多出' if delta>0 else '缺少'} {abs(delta)} 字节")
    print(f"  重对齐后首帧 fid={struct.unpack('<I',data[found:found+4])[0]}")

# 统计整个文件:出现了多少段错位(每次丢/多字节触发一次重找)
segments=0; off=0; misalign_bytes=0
while off+fsize<=N:
    if crc_ok_at(off):
        off+=fsize
    else:
        segments+=1
        # 找下一个对齐点
        nxt=None
        for o in range(off+1, min(off+8192, N-fsize)):
            if crc_ok_at(o) and crc_ok_at(o+fsize):
                nxt=o; break
        if nxt is None:
            break
        misalign_bytes += (nxt-off)
        off=nxt
print(f"\n错位段数={segments} 累计错位字节≈{misalign_bytes}")
