#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TMP_LOW 是逐帧 APPENDFRAME 写的 CSV 温度行(frame_id,tick,temp)。数行连续性。"""
import sys
d = open(sys.argv[1], "rb").read()
fids, bad = [], 0
for ln in d.split(b"\n"):
    ln = ln.strip()
    if not ln:
        continue
    try:
        fids.append(int(ln.split(b",")[0]))
    except Exception:
        bad += 1
if fids:
    span = max(fids) - min(fids) + 1
    uniq = len(set(fids))
    print("rows=%d fid_min=%d fid_max=%d unique=%d span=%d" %
          (len(fids), min(fids), max(fids), uniq, span))
    print("real_missing=%d (%.2f%%)  bad_parse_lines=%d" %
          (span - uniq, 100.0 * (span - uniq) / span, bad))
