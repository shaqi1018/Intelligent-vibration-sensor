#!/usr/bin/env python3
"""只读、非破坏:读卡不同地址区的孤儿数据簇,看高地址区是否真存了BIN帧。
BIN帧22B: frame_id(u32)+ts(u64)+3xint16+crc32。真实数据 frame_id 应单调、CRC应对得上。
若高地址全0/垃圾 → 山寨扩容卡(高地址写不进)。"""
import struct, zlib
f = open(r"\\.\E:", "rb")
b = f.read(512)
bps=struct.unpack_from("<H",b,11)[0]; spc=b[13]; rsvd=struct.unpack_from("<H",b,14)[0]
nfat=b[16]; fatsz=struct.unpack_from("<I",b,36)[0]; totsec=struct.unpack_from("<I",b,32)[0]
data_start=rsvd+nfat*fatsz
def sec_of(cl): return data_start+(cl-2)*spc

def sniff(sec, label):
    f.seek(sec*bps)
    d=f.read(4096)
    nz = sum(1 for x in d if x!=0)
    # 试解析成22B BIN帧,看frame_id是否像单调序号、CRC对不对
    good_crc=0; fids=[]
    for i in range(0, 4096-22, 22):
        fr=d[i:i+22]
        fid=struct.unpack_from("<I",fr,0)[0]
        stored=struct.unpack_from("<I",fr,18)[0]
        if (zlib.crc32(fr[:18])&0xFFFFFFFF)==stored: good_crc+=1
        fids.append(fid)
    print(f"{label}: 扇区{sec}({sec*512//1024//1024}MB) 非零字节{nz}/4096  CRC对{good_crc}帧  前5个fid={fids[:5]}")

print(f"totsec={totsec} ({totsec*512//1024//1024//1024}GB) data_start={data_start}")
# 低地址数据(成功会话2~1的ACC_LOW首簇14) / 中 / 高地址孤儿数据
sniff(sec_of(14),   "低地址(簇14,成功会话数据)")
sniff(sec_of(50000),"中地址(簇5万,~3GB)")
sniff(sec_of(161600),"高地址(簇16.16万,~10GB)")
sniff(sec_of(230000),"极高地址(簇23万,~14GB)")
f.close()
