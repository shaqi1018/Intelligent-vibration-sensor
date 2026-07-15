mic = 351491116 - 44
dur = mic / (96000 * 2)
print("MIC.WAV real duration = %.1f s = %.1f min" % (dur, dur/60))
print()
# (name, nominal_odr, produced_samples_from_ODR_line, ring_drop)
ch = [
    ("LSM_ACC/GYR", 6664, 12242678, 0),
    ("QMA", 1600, 2885307, 4981527),
    ("H3",  400,  726035,  99691),
]
print("%-14s %8s %12s %12s %9s %10s %8s" % ("chan","nomODR","should","produced","realODR","ringDrop","drop%"))
for name, odr, samp, drop in ch:
    should = odr * dur
    real_odr = samp / dur
    # drop% relative to produced+drop (what sensor tried to push into ring)
    total = samp + drop
    dpct = 100.0 * drop / total if total else 0
    print("%-14s %8d %12.0f %12d %9.1f %10d %7.2f%%" % (name, odr, should, samp, real_odr, drop, dpct))
