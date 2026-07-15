import sys

def load_events(p, merge=5.0):
    ts = sorted(float(l.split()[0]) for l in open(p))
    if not ts: return []
    bursts = [[ts[0], ts[0]]]
    for t in ts[1:]:
        if t - bursts[-1][1] <= merge:
            bursts[-1][1] = t
        else:
            bursts.append([t, t])
    return bursts  # list of [start,end]

lsm = load_events(sys.argv[1]); qma = load_events(sys.argv[2]); h3 = load_events(sys.argv[3])

def overlaps(b, arr):
    for a in arr:
        if not (b[1] < a[0]-1.0 or b[0] > a[1]+1.0):
            return True
    return False

print(f"合并间隔5s后的阻塞阵: LSM={len(lsm)} QMA={len(qma)} H3={len(h3)}")
n_all3 = sum(1 for b in lsm if overlaps(b, qma) and overlaps(b, h3))
n_qma  = sum(1 for b in lsm if overlaps(b, qma))
n_h3   = sum(1 for b in lsm if overlaps(b, h3))
print(f"LSM阻塞阵与QMA重叠: {n_qma}/{len(lsm)}")
print(f"LSM阻塞阵与H3重叠 : {n_h3}/{len(lsm)}")
print(f"LSM阻塞阵与QMA且H3都重叠: {n_all3}/{len(lsm)}")
print()
print("LSM阻塞阵窗口 [start-end]s  与QMA/H3是否重叠:")
for b in lsm[:25]:
    print(f"  [{b[0]:7.1f}-{b[1]:7.1f}]  QMA={'Y' if overlaps(b,qma) else '.'} H3={'Y' if overlaps(b,h3) else '.'}")
