#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""按 24B 帧长校验:假设 ARM 下结构体尾部有2字节padding,CRC覆盖前18字节(sizeof-4=20?需试)。
试多种(帧长, crc覆盖长度)组合,找出让CRC全过的正确布局。"""
import sys, zlib, struct
path=sys.argv[1]
data=open(path,"rb").read()

for fsize in (22,24):
    for crclen in (fsize-4, 18, 20):
        if crclen<=0 or crclen>fsize-4: continue
        total=0; bad=0
        n=len(data)//fsize
        for i in range(min(n,200000)):
            fr=data[i*fsize:(i+1)*fsize]
            payload=fr[:crclen]
            stored=struct.unpack("<I",fr[fsize-4:fsize])[0]
            calc=zlib.crc32(payload)&0xFFFFFFFF
            total+=1
            if calc!=stored: bad+=1
        print(f"fsize={fsize} crc_over={crclen} crc_at={fsize-4}: bad={bad}/{total} ({100.0*bad/total:.2f}%)")
