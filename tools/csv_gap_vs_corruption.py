#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""判断 CSV 'true drop' 到底是真丢帧还是字节级损坏/截断的测量伪影。
对每个干净行之间的正跳(gap),看 gap 两端附近是否存在畸形行。
- gap 紧邻畸形行 => 很可能是写块边界把一行/一段切坏,fid跳号是伪影(帧其实写了但被切碎)
- gap 处上下都是干净行 => 真丢帧(帧根本没落盘)
用法: python csv_gap_vs_corruption.py <nfields> <seg1> [seg2 ...]"""
import sys

nfields = int(sys.argv[1])
segs = sys.argv[2:]
MAX_JUMP = 100000

# 读入所有行,标记 clean/bad,记录 fid
rows = []  # (fid_or_None, is_clean)
for path in segs:
    with open(path, "rb") as f:
        for raw in f:
            s = raw.decode("ascii", "replace").strip()
            if s == "" or s.startswith("frame_id") or s.startswith("#"):
                continue
            parts = s.split(",")
            if len(parts) != nfields:
                rows.append((None, False))
                continue
            try:
                fid = int(parts[0])
            except Exception:
                rows.append((None, False))
                continue
            rows.append((fid, True))

# 遍历干净行序列,找 gap;判断该 gap 在 rows 里的位置附近 K 行内是否有畸形
K = 3
clean_idx = [i for i,(fid,ok) in enumerate(rows) if ok]
gap_near_bad = 0      # gap 附近有畸形 => 疑似损坏伪影
gap_clean = 0         # gap 上下纯干净 => 真丢
missing_near_bad = 0
missing_clean = 0
for a_pos, b_pos in zip(clean_idx, clean_idx[1:]):
    fa = rows[a_pos][0]; fb = rows[b_pos][0]
    d = fb - fa
    if d <= 1 or d > MAX_JUMP:
        continue
    miss = d - 1
    # 检查 a_pos..b_pos 之间是否有畸形行,或前后K行
    lo = max(0, a_pos - K)
    hi = min(len(rows), b_pos + K + 1)
    has_bad = any(not rows[i][1] for i in range(lo, hi))
    if has_bad:
        gap_near_bad += 1
        missing_near_bad += miss
    else:
        gap_clean += 1
        missing_clean += miss

print(f"gaps总数={gap_near_bad+gap_clean}")
print(f"  邻近畸形(疑似字节切坏伪影): gaps={gap_near_bad} 缺帧={missing_near_bad:,}")
print(f"  纯干净区(真丢帧):          gaps={gap_clean} 缺帧={missing_clean:,}")
tot = missing_near_bad + missing_clean
if tot:
    print(f"  真丢帧占比 = {100.0*missing_clean/tot:.1f}%  (其余为损坏伪影)")
