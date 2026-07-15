import sys, statistics
ts = [float(l.split()[0]) for l in open(sys.argv[1])]
ev = []; cur = [ts[0]]
for t in ts[1:]:
    if t - cur[-1] <= 1.0:
        cur.append(t)
    else:
        ev.append(cur[0]); cur = [t]
ev.append(cur[0])
d = [ev[i+1]-ev[i] for i in range(len(ev)-1)]
print("event clusters:", len(ev))
print("interval: median=%.1fs mean=%.1fs min=%.1f max=%.1f" % (statistics.median(d), statistics.mean(d), min(d), max(d)))
print("first 20 event times:", [round(x,1) for x in ev[:20]])
