#!/usr/bin/env python3
"""跨通道 gap 时刻对齐检验:若为全局卡阻塞,各通道 gap 应在同一时刻聚集。
读三个 gap 时刻文件(at_sec\tmissing\tdur),检查每个 LSM gap 附近(±win秒)
QMA/H3 是否也有 gap。用法: gap_align.py lsm.txt qma.txt h3.txt [win]"""
import sys

def load(p):
    out = []
    for line in open(p):
        parts = line.split('\t')
        out.append((float(parts[0]), int(parts[1]), float(parts[2])))
    return out

lsm = load(sys.argv[1]); qma = load(sys.argv[2]); h3 = load(sys.argv[3])
win = float(sys.argv[4]) if len(sys.argv) > 4 else 0.5

def near(t, arr, w):
    return [g for g in arr if abs(g[0]-t) <= w]

# 把 LSM gap 按时间聚类(相邻<1s 的算同一次阻塞事件)
def cluster(arr, gap=1.0):
    if not arr: return []
    arr = sorted(arr)
    clusters = [[arr[0]]]
    for g in arr[1:]:
        if g[0] - clusters[-1][-1][0] <= gap:
            clusters[-1].append(g)
        else:
            clusters.append([g])
    return clusters

lc = cluster(lsm)
print(f"LSM: {len(lsm)} gaps -> {len(lc)} 阻塞事件簇")
print(f"{'event_sec':>10} {'lsm_lost':>9} {'qma@±'+str(win):>8} {'h3@±'+str(win):>8}")
both = 0
for c in lc:
    t = c[0][0]
    lsm_lost = sum(g[1] for g in c)
    q = near(t, qma, win); h = near(t, h3, win)
    if q and h: both += 1
    tag = "" if (q and h) else ("  <-- 仅LSM" if not q and not h else "  <-- 部分")
    print(f"{t:10.1f} {lsm_lost:9d} {'Y('+str(sum(g[1] for g in q))+')' if q else 'none':>8} {'Y('+str(sum(g[1] for g in h))+')' if h else 'none':>8}{tag}")
print(f"\n{both}/{len(lc)} 个LSM阻塞事件在QMA和H3同时刻(±{win}s)也丢帧")
