#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""统计 SDwr 日志的写时间预算:把 dma 时间按 cnt=1(单扇区,含元数据) vs 大块(cnt>=64) 分桶,
并把重复固定扇区的 cnt=1(疑似 f_sync 目录/FAT 回写) 单列。
用法: python sdwr_time_budget.py <log.txt>"""
import sys, re, collections

lines = open(sys.argv[1], encoding="utf-8", errors="replace").read().splitlines()
pat = re.compile(r"dma=(\d+)ms.*?cnt=(\d+)\s+sec=(\d+)")
sec_count = collections.Counter()
recs = []
for ln in lines:
    m = pat.search(ln)
    if not m:
        continue
    dma = int(m.group(1)); cnt = int(m.group(2)); sec = int(m.group(3))
    recs.append((dma, cnt, sec))
    if cnt == 1:
        sec_count[sec] += 1

# 重复>=3次的固定扇区视为元数据(FAT/dir 回写)
meta_secs = {s for s,c in sec_count.items() if c >= 3}

t_total = sum(r[0] for r in recs)
t_cnt1_meta = sum(r[0] for r in recs if r[1] == 1 and r[2] in meta_secs)
t_cnt1_data = sum(r[0] for r in recs if r[1] == 1 and r[2] not in meta_secs)
t_big = sum(r[0] for r in recs if r[1] >= 64)
t_mid = sum(r[0] for r in recs if 1 < r[1] < 64)
n_total = len(recs)
n_cnt1_meta = sum(1 for r in recs if r[1] == 1 and r[2] in meta_secs)
n_cnt1_data = sum(1 for r in recs if r[1] == 1 and r[2] not in meta_secs)
n_big = sum(1 for r in recs if r[1] >= 64)
n_mid = sum(1 for r in recs if 1 < r[1] < 64)

print(f"总写命令数={n_total}  总DMA时间={t_total/1000:.1f}s")
print(f"  cnt=1 元数据(重复固定扇区,疑f_sync): 次数={n_cnt1_meta:6d}  时间={t_cnt1_meta/1000:6.1f}s  ({100.0*t_cnt1_meta/t_total:.1f}%)")
print(f"  cnt=1 数据(单扇区小写):              次数={n_cnt1_data:6d}  时间={t_cnt1_data/1000:6.1f}s  ({100.0*t_cnt1_data/t_total:.1f}%)")
print(f"  cnt 2..63 中块:                      次数={n_mid:6d}  时间={t_mid/1000:6.1f}s  ({100.0*t_mid/t_total:.1f}%)")
print(f"  cnt>=64 大块(有效吞吐):              次数={n_big:6d}  时间={t_big/1000:6.1f}s  ({100.0*t_big/t_total:.1f}%)")
print(f"  疑元数据固定扇区数={len(meta_secs)}  top:")
for s,c in sec_count.most_common(8):
    tag = "META" if s in meta_secs else "data"
    print(f"    sec={s} x{c} [{tag}]")
