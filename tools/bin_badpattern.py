#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析坏帧模式:坏帧的 frame_id 是否连续(=纯CRC坏,数据在但校验错),
还是 frame_id 本身跳变(=帧边界错位)。并检查坏帧 payload 与相邻好帧的关系。
用法: python bin_badpattern.py <file.bin> <frame_size>
"""
import sys, zlib, struct

path = sys.argv[1]
fsize = int(sys.argv[2])
data = open(path, "rb").read()
n = len(data) // fsize

def crc_ok(i):
    fr = data[i*fsize:(i+1)*fsize]
    return (zlib.crc32(fr[:-4]) & 0xFFFFFFFF) == struct.unpack("<I", fr[-4:])[0]
def fid(i):
    return struct.unpack("<I", data[i*fsize:i*fsize+4])[0]
# ★2026-07-14 14B 新格式无 ts;仅 22B(MAG/旧)有 ts@4
def ts(i):
    return struct.unpack("<Q", data[i*fsize+4:i*fsize+12])[0] if fsize>=22 else 0

# 统计:坏帧中,frame_id 仍与前一帧连续的比例(=纯CRC问题,边界没错位)
bad_total=0
bad_fid_continuous=0   # 坏帧但 fid == prev_fid+1
bad_fid_broken=0       # 坏帧且 fid 跳变
for i in range(1, n):
    if not crc_ok(i):
        bad_total+=1
        if fid(i)==(fid(i-1)+1)&0xFFFFFFFF:
            bad_fid_continuous+=1
        else:
            bad_fid_broken+=1

print(f"total={n} bad={bad_total}")
print(f"  坏帧中 fid仍连续(数据对位、仅CRC错)= {bad_fid_continuous}")
print(f"  坏帧中 fid跳变(帧边界已错位)      = {bad_fid_broken}")

# 看第一坏帧:逐字节和"假设它没坏、payload=前21字节"对比
# 尝试:坏帧CRC字段(末4B)是否其实是 int16 数据被写进了CRC位置(即帧变短/字段错位)
print("\n前10个坏帧的 (fid, ts, ts-prev_ts):")
cnt=0
for i in range(1, n):
    if not crc_ok(i) and cnt<10:
        print(f"  idx={i} fid={fid(i)} ts={ts(i)} dts={ts(i)-ts(i-1)}")
        cnt+=1
