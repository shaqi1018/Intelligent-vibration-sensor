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
print(f"BytesPerSector = {bps}")
print(f"SectorsPerCluster = {spc}")
print(f"ClusterSize = {bps*spc} bytes = {bps*spc//1024} KB")
print(f"ReservedSectors = {rsvd}, NumFATs = {nfat}")
