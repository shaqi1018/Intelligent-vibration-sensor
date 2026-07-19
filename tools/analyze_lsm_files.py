import os
import sys

session_dir = sys.argv[1] if len(sys.argv) > 1 else 'E:/CTBX_2026-07-19-20-46'

print("=== LSM 文件分析 ===\n")

# 查看所有 ACC_LOW 和 GYR_LOW 文件
acc_files = sorted([f for f in os.listdir(session_dir) if f.startswith('ACC_LOW') and f.endswith('.CSV')])
gyr_files = sorted([f for f in os.listdir(session_dir) if f.startswith('GYR_LOW') and f.endswith('.CSV')])

print("ACC_LOW 文件:")
acc_total_bytes = 0
acc_total_lines = 0
for f in acc_files:
    path = os.path.join(session_dir, f)
    size = os.path.getsize(path)
    acc_total_bytes += size
    with open(path, 'r', encoding='utf-8') as fp:
        lines = sum(1 for _ in fp) - 1  # -1 for header
    acc_total_lines += lines
    print(f"  {f}: {size/1024/1024:.1f}MB, {lines:,} lines")

print(f"\nACC_LOW 总计: {acc_total_bytes/1024/1024:.1f}MB, {acc_total_lines:,} lines")

print("\nGYR_LOW 文件:")
gyr_total_bytes = 0
gyr_total_lines = 0
for f in gyr_files:
    path = os.path.join(session_dir, f)
    size = os.path.getsize(path)
    gyr_total_bytes += size
    try:
        with open(path, 'r', encoding='utf-8') as fp:
            lines = sum(1 for _ in fp) - 1
        gyr_total_lines += lines
        print(f"  {f}: {size/1024/1024:.1f}MB, {lines:,} lines")
    except Exception as e:
        print(f"  {f}: {size/1024/1024:.1f}MB, 读取错误: {e}")

print(f"\nGYR_LOW 总计: {gyr_total_bytes/1024/1024:.1f}MB, {gyr_total_lines:,} lines (如果读取成功)")

# 分析时间跨度
print("\n=== 时间跨度分析 ===")
interval_us = 150  # LSM 间隔
if acc_total_lines > 0:
    acc_duration_s = acc_total_lines * interval_us / 1e6
    print(f"ACC_LOW 覆盖时长: {acc_duration_s:.0f}s = {acc_duration_s/60:.1f}min")
if gyr_total_lines > 0:
    gyr_duration_s = gyr_total_lines * interval_us / 1e6
    print(f"GYR_LOW 覆盖时长: {gyr_duration_s:.0f}s = {gyr_duration_s/60:.1f}min")

# 文件切换时间点
print("\n=== 文件创建时间 ===")
for f in acc_files + gyr_files:
    path = os.path.join(session_dir, f)
    mtime = os.path.getmtime(path)
    from datetime import datetime
    dt = datetime.fromtimestamp(mtime)
    print(f"  {f}: {dt.strftime('%H:%M:%S')}")
