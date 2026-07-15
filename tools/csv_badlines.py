#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""扫描 CSV,统计坏行:字段数不符/含非法字符/datetime粘连。区分中间坏行(写路径问题)
vs 末尾单行(掉电截断,可接受)。用法: python csv_badlines.py <file.csv> <expected_fields>"""
import sys
path=sys.argv[1]; expect=int(sys.argv[2])
total=0; bad=0; bad_samples=[]; last_bad_line=-1
with open(path,"rb") as f:
    data=f.read()
text=data.decode("ascii","replace")
lines=text.split("\n")
for i,ln in enumerate(lines):
    ln=ln.rstrip("\r")
    if ln=="": continue
    total+=1
    parts=ln.split(",")
    ok = (len(parts)==expect)
    if ok:
        for p in parts:
            # 每字段应是数字(可含负号/小数点)
            s=p.strip()
            if s=="" or any(c not in "0123456789.-" for c in s):
                ok=False; break
    if not ok:
        bad+=1; last_bad_line=i
        if len(bad_samples)<8: bad_samples.append((i,ln[:60]))
print(f"file={path}")
print(f"总行={total} 坏行={bad} 末坏行号={last_bad_line} 总行号={len(lines)-1}")
print(f"  末坏行是否=最后一行(掉电截断特征): {last_bad_line>=len(lines)-2}")
for n,s in bad_samples:
    print(f"  行{n}: {s}")
