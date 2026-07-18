#!/usr/bin/env python3
"""读 E: 卷的 BPB 引导扇区,算 FAT32 簇大小。"""
import struct
# Windows 原始卷路径
path = r"\\.\E:"
with open(path, "rb") as f:
    b = f.read(512)
bps = struct.unpack_from("<H", b, 11)[0]   # BytesPerSector @0x0B
spc = b[13]                                 # SectorsPerCluster @0x0D
rsvd = struct.unpack_from("<H", b, 14)[0]
nfat = b[16]
fatsz32 = struct.unpack_from("<I", b, 36)[0]
tot_sec32 = struct.unpack_from("<I", b, 32)[0]
data_start = rsvd + nfat * fatsz32   # 相对分区起始的扇区
print(f"BytesPerSector = {bps}")
print(f"SectorsPerCluster = {spc}")
print(f"ClusterSize = {bps*spc} bytes = {bps*spc//1024} KB")
print(f"ReservedSectors = {rsvd}, NumFATs = {nfat}, FATSz32 = {fatsz32}")
print(f"数据区起始扇区(相对分区) = {data_start}")
print(f"  mod 128(64KB簇) = {data_start % 128}   (0=完美对齐)")
print(f"  mod 32(16KB)    = {data_start % 32}")
print(f"TotSec32 = {tot_sec32} (~{tot_sec32*512//(1024*1024*1024)}GB, 仅分区内)")
# 注意:还要加分区在整盘的起始偏移(MBR)。卷设备 \\.\E: 从分区起点算,
# 但卡的物理擦除边界是相对整盘 LBA。日志 sec 是 FatFs 给的分区内扇区号。
