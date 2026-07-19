import json
import sys
import os
import glob

session_dir = sys.argv[1] if len(sys.argv) > 1 else 'E:/CTBX_2026-07-19-17-48'

with open(f'{session_dir}/CONFIG.JSN', 'r', encoding='utf-8') as f:
    cfg = json.load(f)

duration = cfg['timebase']['duration_s']
print(f"Duration: {duration}s = {duration/60:.1f}min")

# Calculate expected samples
intervals = cfg['timebase']['interval_us']
expected = {}
expected['LSM'] = duration * 1e6 / intervals['accel_low']
expected['QMA'] = duration * 1e6 / intervals['accel_mid']
expected['H3'] = duration * 1e6 / intervals['accel_high']
expected['MAG'] = duration * 1e6 / intervals['mag']

print("\nExpected samples:")
for sensor, exp in expected.items():
    print(f"  {sensor}: {exp:,.0f}")

# Count actual samples from ALL CSV files (支持分段文件)
file_patterns = {
    'LSM': 'ACC_LOW*.CSV',
    'QMA': 'ACC_MID*.CSV',
    'H3': 'ACC_HIGH*.CSV',
    'MAG': 'MAG*.CSV',
}

print("\nActual samples and drop rates:")
actual = {}
for sensor, pattern in file_patterns.items():
    # 查找所有匹配的文件
    files = sorted(glob.glob(f'{session_dir}/{pattern}'))
    if not files:
        print(f"  {sensor}: No files found")
        continue

    total_lines = 0
    for fpath in files:
        try:
            with open(fpath, 'r', encoding='utf-8') as f:
                lines = sum(1 for _ in f) - 1  # -1 for header
            total_lines += lines
        except UnicodeDecodeError as e:
            print(f"  {sensor}: Error reading {os.path.basename(fpath)} - {e}")
            continue

    actual[sensor] = total_lines
    drop_pct = (1 - total_lines / expected[sensor]) * 100
    num_files = len(files)
    if num_files > 1:
        print(f"  {sensor}: {total_lines:,} actual ({drop_pct:.1f}% drop) [{num_files} files]")
    else:
        print(f"  {sensor}: {total_lines:,} actual ({drop_pct:.1f}% drop)")
