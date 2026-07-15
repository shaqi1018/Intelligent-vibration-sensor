#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按字段解析给定绝对偏移处连续几帧,并对每帧独立算CRC看是否通过,帮助定位错位从哪个字节开始。
★2026-07-14 帧瘦身:新格式 14B = fid(4) x2 y2 z2 crc(4),无 ts。旧/MAG 格式 22B = fid(4) ts(8) x2 y2 z2 crc(4)。
用法: bin_fields.py <path> <start_off> <count> [fsz=14]"""
import sys, zlib, struct
path=sys.argv[1]; start=int(sys.argv[2]); count=int(sys.argv[3])
fsz=int(sys.argv[4]) if len(sys.argv)>4 else 14
data=open(path,"rb").read()
off=start
for k in range(count):
    fr=data[off:off+fsz]
    if len(fr)<fsz: break
    if fsz==14:   # 新格式:无 ts
        fid=struct.unpack_from("<I",fr,0)[0]
        x,y,z=struct.unpack_from("<hhh",fr,4)
        crc=struct.unpack_from("<I",fr,10)[0]
        calc=zlib.crc32(fr[:10])&0xFFFFFFFF
        print(f"off={off} fid={fid} xyz=({x},{y},{z}) crc={crc:08X} calc={calc:08X} {'OK' if crc==calc else 'BAD'}")
    else:         # 22B:含 ts(MAG 或旧数据)
        fid,ts=struct.unpack_from("<IQ",fr,0)
        x,y,z=struct.unpack_from("<hhh",fr,12)
        crc=struct.unpack_from("<I",fr,18)[0]
        calc=zlib.crc32(fr[:18])&0xFFFFFFFF
        print(f"off={off} fid={fid} ts={ts} xyz=({x},{y},{z}) crc={crc:08X} calc={calc:08X} {'OK' if crc==calc else 'BAD'}")
    off+=fsz
