#!/usr/bin/env python3
"""检查 E: 的 FAT 一致性:FAT1 vs FAT2 是否一致;会话目录簇161491的FAT项;
以及161491周边簇的分配情况(看是否被数据覆盖)。"""
import struct
f = open(r"\\.\E:", "rb")
b = f.read(512)
bps = struct.unpack_from("<H", b, 11)[0]
spc = b[13]
rsvd = struct.unpack_from("<H", b, 14)[0]
nfat = b[16]
fatsz = struct.unpack_from("<I", b, 36)[0]
totsec = struct.unpack_from("<I", b, 32)[0]
print(f"bps={bps} spc={spc} rsvd={rsvd} nfat={nfat} fatsz={fatsz} totsec={totsec}")
fat1_base = rsvd * bps
fat2_base = (rsvd + fatsz) * bps

def read_fat_entry(fat_base, cl):
    off = fat_base + cl*4
    sec = (off//bps)*bps
    f.seek(sec); buf=f.read(bps)
    return struct.unpack_from("<I", buf, off-sec)[0] & 0x0FFFFFFF

# 1) FAT1 vs FAT2 一致性(抽查会话目录簇及周边)
print("\n=== 会话目录簇 161491 及周边 FAT 项 (FAT1 / FAT2) ===")
for cl in range(161488, 161496):
    v1 = read_fat_entry(fat1_base, cl)
    v2 = read_fat_entry(fat2_base, cl)
    mark = "" if v1==v2 else "  <<< FAT1!=FAT2 不一致!"
    print(f"  clus {cl}: FAT1=0x{v1:08X} FAT2=0x{v2:08X}{mark}")

# 2) FAT1整体和FAT2整体差异计数(抽样每1000簇)
print("\n=== FAT1 vs FAT2 全局抽样差异(每512簇取1) ===")
ndata = (totsec-(rsvd+nfat*fatsz))//spc
diff=0; checked=0
for cl in range(2, min(ndata+2, 240000), 512):
    if read_fat_entry(fat1_base,cl)!=read_fat_entry(fat2_base,cl): diff+=1
    checked+=1
print(f"  抽查{checked}簇, 不一致{diff}处")
f.close()
