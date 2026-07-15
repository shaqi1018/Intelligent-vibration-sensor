#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
path=sys.argv[1]; start=int(sys.argv[2]); length=int(sys.argv[3])
data=open(path,"rb").read()
seg=data[start:start+length]
for k in range(0,len(seg),22):
    print(f"  +{k:4d} (off {start+k}): {seg[k:k+22].hex()}")
print("flat:", seg.hex())
