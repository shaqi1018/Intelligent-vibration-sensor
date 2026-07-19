import sys
import re

log_file = sys.argv[1] if len(sys.argv) > 1 else 'c:/Users/Q1/Desktop/csv.txt'

with open(log_file, 'r', encoding='utf-8') as f:
    lines = [l.strip() for l in f if '[SDwr]' in l]

total = len(lines)
cnt1 = sum(1 for l in lines if 'cnt=1 ' in l)
cnt128 = sum(1 for l in lines if 'cnt=128 ' in l)
cnt127 = sum(1 for l in lines if 'cnt=127 ' in l)
other = total - cnt1 - cnt128 - cnt127

print(f"Total SDwr lines: {total}")
print(f"  cnt=1:   {cnt1:3d} ({cnt1/total*100:5.1f}%)")
print(f"  cnt=128: {cnt128:3d} ({cnt128/total*100:5.1f}%)")
print(f"  cnt=127: {cnt127:3d} ({cnt127/total*100:5.1f}%)")
print(f"  other:   {other:3d} ({other/total*100:5.1f}%)")

# Analyze cnt=1 sectors
cnt1_lines = [l for l in lines if 'cnt=1 ' in l]
sectors = []
for line in cnt1_lines:
    m = re.search(r'sec=(\d+)', line)
    if m:
        sectors.append(int(m.group(1)))

print(f"\nAnalyzing {len(sectors)} cnt=1 sectors:")
small_sec = [s for s in sectors if s < 100000]
large_sec = [s for s in sectors if s >= 100000]
print(f"  Small sectors (<100000): {len(small_sec)} ({len(small_sec)/len(sectors)*100:.1f}%) - FAT/directory")
print(f"    Unique sectors: {len(set(small_sec))}")
print(f"    Most frequent: {sorted(set(small_sec), key=small_sec.count, reverse=True)[:10]}")
print(f"  Large sectors (>=100000): {len(large_sec)} ({len(large_sec)/len(sectors)*100:.1f}%) - data area")
print(f"    Unique sectors: {len(set(large_sec))}")

# Time cost analysis
print(f"\nTime cost analysis:")
cnt1_time = cnt1 * 130  # Average ~130ms per cnt=1
cnt128_time = cnt128 * 140  # Average ~140ms per cnt=128
total_time = cnt1_time + cnt128_time
print(f"  cnt=1 total time:   {cnt1_time/1000:.1f}s ({cnt1_time/total_time*100:.1f}% of write time)")
print(f"  cnt=128 total time: {cnt128_time/1000:.1f}s ({cnt128_time/total_time*100:.1f}% of write time)")
print(f"  Total write time:   {total_time/1000:.1f}s")
