#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对每个错位段:定位精确的插入点,检查是否=512扇区边界,且插入的2字节是否是
边界前2字节的重复。统计规律。"""
import sys, zlib, struct
path=sys.argv[1]; fsize=22
data=open(path,"rb").read(); N=len(data)
def ok(o):
    if o+fsize>N: return False
    fr=data[o:o+fsize]; return (zlib.crc32(fr[:-4])&0xFFFFFFFF)==struct.unpack("<I",fr[-4:])[0]

off=0; checked=0; at_sector=0; dup2=0; ins_amounts={}
samples=[]
while off+fsize<=N and checked<3000:
    if ok(off): off+=fsize; continue
    # 找重对齐点 & 插入字节数
    nxt=None
    for d in range(1,64):
        if ok(off+d) and ok(off+d+fsize): nxt=off+d; break
    if nxt is None: off+=1; continue
    delta=nxt-off
    ins = delta - fsize   # 相对22多出的字节(正=插入)
    ins_amounts[ins]=ins_amounts.get(ins,0)+1
    if ins>0:
        # 插入点:在 off..nxt 之间某处。扫描找到512边界
        # 边界候选:off..nxt 范围内 512 的倍数
        for b in range((off//512)*512, nxt+512, 512):
            if off < b <= nxt:
                # 检查 b 处前 ins 字节 == b 处后 ins 字节(重复)
                if data[b-ins:b]==data[b:b+ins]:
                    dup2+=1
                at_sector+=1
                if len(samples)<5:
                    samples.append((off,b,ins,data[b-ins-2:b].hex(),data[b:b+ins+2].hex()))
                break
    checked+=1
    off=nxt

print(f"检查错位段={checked}")
print(f"  插入字节量分布: {dict(sorted(ins_amounts.items()))}")
print(f"  插入点落在512扇区边界的={at_sector}/{checked}")
print(f"  且插入字节=边界前字节重复的={dup2}/{at_sector}")
for o,b,ins,before,after in samples:
    print(f"  坏点off={o} 扇区边界={b}(={b//512}*512) 插入{ins}B 边界前={before} 边界后={after}")
