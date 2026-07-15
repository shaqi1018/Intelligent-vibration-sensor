#!/usr/bin/env python3
"""dump 指定簇的前 512 字节原始 hex,看目录簇到底有没有内容。"""
import struct, sys
path = r"\\.\E:"
f = open(path, "rb")
b = f.read(512)
bps = struct.unpack_from("<H", b, 11)[0]
spc = b[13]
rsvd = struct.unpack_from("<H", b, 14)[0]
nfat = b[16]
fatsz = struct.unpack_from("<I", b, 36)[0]
data_start = rsvd + nfat * fatsz
clbytes = bps * spc

def read_fat(cl):
    byte_off = rsvd*bps + cl*4
    sec=(byte_off//bps)*bps
    f.seek(sec); buf=f.read(bps)
    return struct.unpack_from("<I", buf, byte_off-sec)[0] & 0x0FFFFFFF

for cl in [int(x) for x in sys.argv[1:]]:
    sec = data_start + (cl-2)*spc
    f.seek(sec*bps)
    d = f.read(512)
    fatval = read_fat(cl)
    print(f"=== cluster {cl} (sector {sec}) FATentry=0x{fatval:08X} ===")
    # first 4 dir entries (32B each)
    for i in range(0,128,32):
        e=d[i:i+32]
        nm=''.join(chr(c) if 32<=c<127 else '.' for c in e[0:11])
        print(f"  [{i//32}] name='{nm}' attr=0x{e[11]:02X} raw0=0x{e[0]:02X}")
    # hex of first 96 bytes
    print("  hex:", d[:96].hex())
f.close()
