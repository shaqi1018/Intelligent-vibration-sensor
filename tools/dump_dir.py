#!/usr/bin/env python3
"""读 E: 卷根目录,找 CTBX 会话目录的目录项,dump 它的子目录项(文件名/大小/首簇)。
判断是"文件目录项没写(空目录)"还是"目录项在但大小/簇错(FAT问题)"。"""
import struct

path = r"\\.\E:"
f = open(path, "rb")
b = f.read(512)
bps = struct.unpack_from("<H", b, 11)[0]
spc = b[13]
rsvd = struct.unpack_from("<H", b, 14)[0]
nfat = b[16]
fatsz = struct.unpack_from("<I", b, 36)[0]   # FAT32 sectors per FAT @0x24
rootclus = struct.unpack_from("<I", b, 44)[0]  # root dir first cluster @0x2C
print(f"bps={bps} spc={spc} rsvd={rsvd} nfat={nfat} fatsz={fatsz} rootclus={rootclus}")

data_start = rsvd + nfat * fatsz   # first data sector (cluster 2)
clbytes = bps * spc

def clus_to_sec(cl):
    return data_start + (cl - 2) * spc

def read_cluster(cl):
    f.seek(clus_to_sec(cl) * bps)
    return f.read(clbytes)

def read_fat(cl):
    byte_off = rsvd * bps + cl * 4
    sec = (byte_off // bps) * bps          # sector-aligned base
    f.seek(sec)
    buf = f.read(bps)
    within = byte_off - sec
    return struct.unpack_from("<I", buf, within)[0] & 0x0FFFFFFF

def parse_dir(data, label):
    print(f"\n--- {label} entries ---")
    n = 0
    for i in range(0, len(data), 32):
        e = data[i:i+32]
        if e[0] == 0x00:
            break
        if e[0] == 0xE5:
            continue
        attr = e[11]
        if attr == 0x0F:   # LFN entry
            continue
        name = ''.join(chr(c) if 32 <= c < 127 else '.' for c in e[0:11])
        fst = (struct.unpack_from("<H", e, 20)[0] << 16) | struct.unpack_from("<H", e, 26)[0]
        sz = struct.unpack_from("<I", e, 28)[0]
        print(f"  name='{name}' attr=0x{attr:02X} firstclus={fst} size={sz}")
        n += 1
    print(f"  ({n} real entries)")

# root dir: follow cluster chain
cl = rootclus
root = b""
for _ in range(20):
    root += read_cluster(cl)
    nx = read_fat(cl)
    if nx >= 0x0FFFFFF8 or nx == 0: break
    cl = nx
parse_dir(root, "ROOT")

# find CTBX dir first cluster from root
ctbx_cl = None
for i in range(0, len(root), 32):
    e = root[i:i+32]
    if e[0] in (0x00,0xE5) or e[11]==0x0F: continue
    nm = ''.join(chr(c) if 32 <= c < 127 else '.' for c in e[0:11])
    if nm.startswith('CTBX'):
        ctbx_cl = (struct.unpack_from("<H", e, 20)[0] << 16) | struct.unpack_from("<H", e, 26)[0]
        print(f"\nFound CTBX dir '{nm}' firstclus={ctbx_cl}")

# dump ALL CTBX dirs found in root
ctbx_list = []
for i in range(0, len(root), 32):
    e = root[i:i+32]
    if e[0] in (0x00,0xE5) or e[11]==0x0F or (e[11] & 0x10)==0: continue
    nm = ''.join(chr(c) if 32 <= c < 127 else '.' for c in e[0:11])
    if nm.startswith('CTBX'):
        fc = (struct.unpack_from("<H", e, 20)[0] << 16) | struct.unpack_from("<H", e, 26)[0]
        ctbx_list.append((nm, fc))

for nm, fc in ctbx_list:
    cl = fc
    d = b""
    steps = 0
    chain = []
    for _ in range(50):
        d += read_cluster(cl)
        chain.append(cl)
        nx = read_fat(cl)
        steps += 1
        if nx >= 0x0FFFFFF8 or nx == 0 or nx == 1: break
        cl = nx
    print(f"\n### CTBX '{nm}' firstclus={fc} chain={chain[:8]}{'...' if len(chain)>8 else ''} ({steps} clus)")
    parse_dir(d, nm)
f.close()
