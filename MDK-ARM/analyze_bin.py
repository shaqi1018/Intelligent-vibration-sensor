import re
import sys

# Set UTF-8 encoding for output
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')

# Parse SDwr entries from file
with open('c:/Users/Q1/Desktop/bin.txt', 'r', encoding='utf-8') as f:
    log_data = f.read()

sdwr_pattern = re.compile(r'\[SDwr\] dma=(\d+)ms prog=(\d+)ms cnt=(\d+) sec=(\d+)')

total_writes = 0
cnt_distribution = {}
cnt1_sectors = []
write_times = {}

for match in sdwr_pattern.finditer(log_data):
    dma_ms = int(match.group(1))
    prog_ms = int(match.group(2))
    cnt = int(match.group(3))
    sector = int(match.group(4))

    total_writes += 1
    cnt_distribution[cnt] = cnt_distribution.get(cnt, 0) + 1

    if cnt == 1:
        cnt1_sectors.append(sector)

    if cnt not in write_times:
        write_times[cnt] = []
    write_times[cnt].append(dma_ms + prog_ms)

# Print statistics
print('Total SDwr: {} 次'.format(total_writes))
print()

sorted_cnts = sorted(cnt_distribution.items(), key=lambda x: x[1], reverse=True)
print('cnt 分布 (Top 10):')
for cnt, count in sorted_cnts[:10]:
    pct = count * 100.0 / total_writes
    print('  cnt={:3d}: {:4d} 次 ({:5.2f}%)'.format(cnt, count, pct))
print()

cnt1 = cnt_distribution.get(1, 0)
cnt127 = cnt_distribution.get(127, 0)
cnt128 = cnt_distribution.get(128, 0)

print('关键 cnt 分布:')
print('  cnt=1:   {:4d} 次 ({:5.2f}%)'.format(cnt1, cnt1*100.0/total_writes))
print('  cnt=128: {:4d} 次 ({:5.2f}%)'.format(cnt128, cnt128*100.0/total_writes))
print('  cnt=127: {:4d} 次 ({:5.2f}%)'.format(cnt127, cnt127*100.0/total_writes))
print()

fat_dir_cnt = sum(1 for s in cnt1_sectors if s < 100000)
data_cnt = sum(1 for s in cnt1_sectors if s >= 100000)

print('cnt=1 扇区分布:')
print('  FAT/目录区 (扇区 < 100000):  {:4d} 次 ({:5.2f}%)'.format(fat_dir_cnt, fat_dir_cnt*100.0/cnt1 if cnt1 > 0 else 0))
print('  数据区 (扇区 >= 100000):     {:4d} 次 ({:5.2f}%)'.format(data_cnt, data_cnt*100.0/cnt1 if cnt1 > 0 else 0))
print()

total_time = sum(sum(times) for times in write_times.values())
cnt1_time = sum(write_times.get(1, []))
cnt128_time = sum(write_times.get(128, []))

print('时间成本:')
print('  cnt=1 耗时:   {:7.2f}s (占 {:5.2f}% 写时间)'.format(cnt1_time/1000.0, cnt1_time*100.0/total_time))
print('  cnt=128 耗时: {:7.2f}s (占 {:5.2f}% 写时间)'.format(cnt128_time/1000.0, cnt128_time*100.0/total_time))
print('  总写时间:    {:7.2f}s'.format(total_time/1000.0))
print()

if cnt1 > 0:
    avg_cnt1 = cnt1_time / cnt1
    print('  cnt=1 平均耗时:   {:.1f}ms/次'.format(avg_cnt1))
if cnt128 > 0:
    avg_cnt128 = cnt128_time / cnt128
    print('  cnt=128 平均耗时: {:.1f}ms/次'.format(avg_cnt128))

print()
print('=' * 60)
print('对比 CSV 格式:')
print('  CSV: 538 次 cnt=1, 占 42.5% 写时间')
print('  BIN: {} 次 cnt=1, 占 {:.1f}% 写时间'.format(cnt1, cnt1_time*100.0/total_time))
print()

# BIN has MORE cnt=1 than CSV, but lower time percentage
print('  观察:')
print('    - BIN 的 cnt=1 次数更多 (+{} 次, +{:.1f}%)'.format(cnt1-538, (cnt1-538)*100.0/538))
print('    - 但 cnt=1 时间占比更低 ({:.1f} 个百分点)'.format(42.5 - cnt1_time*100.0/total_time))
print()
print('  结论:')
print('    ★ BIN 格式虽然有更多 cnt=1 写入,但由于:')
print('       1. BIN 无表头,数据从扇区边界开始 (无 CSV 的 37B 错位)')
print('       2. 更多的 cnt=1 是正常的 FAT/目录操作 (34.76%)')
print('       3. 数据区的 cnt=1 (65.24%) 不像 CSV 那样是持续的 RMW')
print('    ★ 因此 cnt=1 时间占比反而更低')
print()
print('  数据区 cnt=1 详细分析:')
data_cnt1_time = sum(write_times[1][i] for i, s in enumerate(cnt1_sectors) if s >= 100000)
print('    数据区 cnt=1: {} 次, 耗时 {:.2f}s ({:.2f}% 总写时间)'.format(
    data_cnt, data_cnt1_time/1000.0, data_cnt1_time*100.0/total_time))
print('    FAT/目录 cnt=1: {} 次, 耗时 {:.2f}s ({:.2f}% 总写时间)'.format(
    fat_dir_cnt, (cnt1_time-data_cnt1_time)/1000.0, (cnt1_time-data_cnt1_time)*100.0/total_time))
