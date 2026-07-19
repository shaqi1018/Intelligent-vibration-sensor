import json
import sys

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

# Count actual samples from CSV files
files = {
    'LSM': f'{session_dir}/ACC_LOW001.CSV',
    'QMA': f'{session_dir}/ACC_MID001.CSV',
    'H3': f'{session_dir}/ACC_HIGH001.CSV',
    'MAG': f'{session_dir}/MAG001.CSV',
}

print("\nActual samples and drop rates:")
actual = {}
for sensor, path in files.items():
    with open(path, 'r', encoding='utf-8') as f:
        lines = sum(1 for _ in f) - 1  # -1 for header
    actual[sensor] = lines
    drop_pct = (1 - lines / expected[sensor]) * 100
    print(f"  {sensor}: {lines:,} actual ({drop_pct:.1f}% drop)")
