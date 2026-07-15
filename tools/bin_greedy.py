#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""贪婪解析:总是在当前位置尝试22B帧,CRC过就前进22,不过就逐字节滑动找下一个CRC过的帧。
统计:成功帧数、丢弃字节、fid是否单调连续(真实丢帧 vs 解析错位)。"""
import sys, zlib, struct
path=sys.argv[1]; fsize=22
data=open(path,"rb").read(); N=len(data)
def ok(o):
    if o+fsize>N: return False
    fr=data[o:o+fsize]; return (zlib.crc32(fr[:-4])&0xFFFFFFFF)==struct.unpack("<I",fr[-4:])[0]
def fid(o): return struct.unpack("<I",data[o:o+4])[0]

off=0; good=0; skipped=0; fids=[]
while off+fsize<=N:
    if ok(off):
        fids.append(fid(off)); good+=1; off+=fsize
    else:
        off+=1; skipped+=1
print(f"贪婪解析: 好帧={good} 跳过字节={skipped} 总字节={N}")
# fid 连续性
jumps=0; back=0; maxfid=fids[0] if fids else 0
for i in range(1,len(fids)):
    d=fids[i]-fids[i-1]
    if d!=1:
        jumps+=1
        if d<0: back+=1
print(f"好帧fid: 首={fids[0]} 末={fids[-1]} 期望帧数={fids[-1]-fids[0]+1} 实得={good}")
print(f"  fid跳变(非+1)={jumps} 其中回退={back}")
print(f"  => 若跳变≈0且实得≈期望: 数据完整,之前的'坏'纯是固定22对齐假设错误")
