#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""计算CSV vs BIN理论产出速率，验证636/425 KB/s的说法"""

# 传感器配置 (满配)
sensors = {
    # (name, ODR_Hz, CSV_bytes_per_row, BIN_bytes_per_frame)
    "ACC_LOW (LSM加速度)": (6664, None, 14),  # CSV行长待实测
    "GYR_LOW (LSM角速度)": (6664, None, 14),
    "ACC_MID (QMA)":       (1600, None, 14),
    "ACC_HIGH (H3)":       (400,  None, 14),
    "MAG (磁力计)":         (100,  None, 22),
    "ENV (温湿度)":         (1,    None, 24),
}

# MIC (96kHz采样率，48kHz时减半)
mic_96k = 96000 * 2  # 96kHz × 2 bytes = 192000 bytes/s
mic_48k = 48000 * 2  # 48kHz × 2 bytes = 96000 bytes/s

print("=" * 80)
print("理论产出速率计算 (CSV vs BIN)")
print("=" * 80)

# CSV格式分析
print("\n【CSV格式】")
print("假设每行格式: frame_id,datetime,x,y,z\\r\\n")
print("  - frame_id: ~10位数字 = 10B")
print("  - datetime: 12B (YYMMDDHHMMSS)")
print("  - 逗号×4 + \\r\\n = 6B")
print("  - x,y,z: 3个浮点数，每个~5-7位 ≈ 18B (6B×3)")
print("  总计: 10+12+6+18 = 46B/行 (含datetime)")
print("  去datetime后: 10+6+18 = 34B/行")
print()

# 使用43B (从CSV slim文档的"ACC_LOW 行 43→30B")
csv_acc_with_dt = 43  # bytes/row with datetime
csv_acc_no_dt = 30    # bytes/row without datetime

print(f"从文档: ACC_LOW行长 = {csv_acc_with_dt}B (含datetime) → {csv_acc_no_dt}B (去datetime)")
print(f"datetime占比 = {csv_acc_with_dt - csv_acc_no_dt}B = {(csv_acc_with_dt - csv_acc_no_dt)/csv_acc_with_dt*100:.1f}%")
print()

# 计算各传感器CSV产出 (含datetime)
print("含datetime的CSV产出:")
csv_total = 0
for name, (odr, _, _) in sensors.items():
    if "LSM" in name or "QMA" in name or "H3" in name:
        bytes_per_row = csv_acc_with_dt  # 加速度/角速度用43B
    elif "MAG" in name:
        bytes_per_row = csv_acc_with_dt  # 同样格式
    elif "ENV" in name:
        bytes_per_row = csv_acc_with_dt  # 同样格式
    else:
        bytes_per_row = csv_acc_with_dt

    throughput = odr * bytes_per_row
    csv_total += throughput
    print(f"  {name:25s}: {odr:5d} Hz × {bytes_per_row:2d} B = {throughput/1024:7.1f} KB/s")

csv_total_with_mic96 = csv_total + mic_96k
csv_total_with_mic48 = csv_total + mic_48k

print(f"  {'MIC (96kHz)':25s}: 96000 Hz × 2 B = {mic_96k/1024:7.1f} KB/s")
print(f"  {'-'*60}")
print(f"  {'总计 (96kHz mic)':25s}: {csv_total_with_mic96/1024:7.1f} KB/s")
print()
print(f"  {'MIC (48kHz)':25s}: 48000 Hz × 2 B = {mic_48k/1024:7.1f} KB/s")
print(f"  {'-'*60}")
print(f"  {'总计 (48kHz mic)':25s}: {csv_total_with_mic48/1024:7.1f} KB/s")

# 去datetime后的CSV产出
print("\n去datetime后的CSV产出:")
csv_slim_total = 0
for name, (odr, _, _) in sensors.items():
    if "LSM" in name or "QMA" in name or "H3" in name:
        bytes_per_row = csv_acc_no_dt  # 30B
    elif "MAG" in name:
        # MAG去datetime: 43B → 30B (同比例)
        bytes_per_row = csv_acc_no_dt
    elif "ENV" in name:
        bytes_per_row = csv_acc_no_dt
    else:
        bytes_per_row = csv_acc_no_dt

    throughput = odr * bytes_per_row
    csv_slim_total += throughput
    print(f"  {name:25s}: {odr:5d} Hz × {bytes_per_row:2d} B = {throughput/1024:7.1f} KB/s")

csv_slim_with_mic96 = csv_slim_total + mic_96k
csv_slim_with_mic48 = csv_slim_total + mic_48k

print(f"  {'MIC (96kHz)':25s}: 96000 Hz × 2 B = {mic_96k/1024:7.1f} KB/s")
print(f"  {'-'*60}")
print(f"  {'总计 (96kHz mic)':25s}: {csv_slim_with_mic96/1024:7.1f} KB/s")
print()
print(f"  {'MIC (48kHz)':25s}: 48000 Hz × 2 B = {mic_48k/1024:7.1f} KB/s")
print(f"  {'-'*60}")
print(f"  {'总计 (48kHz mic)':25s}: {csv_slim_with_mic48/1024:7.1f} KB/s")

# BIN格式计算
print("\n【BIN格式】")
print("每帧格式: frame_id(4B) + data(6-18B) + CRC32(4B)")
bin_total = 0
for name, (odr, _, bin_size) in sensors.items():
    throughput = odr * bin_size
    bin_total += throughput
    print(f"  {name:25s}: {odr:5d} Hz × {bin_size:2d} B = {throughput/1024:7.1f} KB/s")

bin_total_with_mic96 = bin_total + mic_96k
bin_total_with_mic48 = bin_total + mic_48k

print(f"  {'MIC (96kHz)':25s}: 96000 Hz × 2 B = {mic_96k/1024:7.1f} KB/s")
print(f"  {'-'*60}")
print(f"  {'总计 (96kHz mic)':25s}: {bin_total_with_mic96/1024:7.1f} KB/s")
print()
print(f"  {'MIC (48kHz)':25s}: 48000 Hz × 2 B = {mic_48k/1024:7.1f} KB/s")
print(f"  {'-'*60}")
print(f"  {'总计 (48kHz mic)':25s}: {bin_total_with_mic48/1024:7.1f} KB/s")

# 对比
print("\n" + "=" * 80)
print("对比总结")
print("=" * 80)
print(f"{'格式':<20s} {'96kHz mic':<15s} {'48kHz mic':<15s} {'节省 (vs CSV+dt)'}")
print("-" * 80)
print(f"{'CSV (含datetime)':<20s} {csv_total_with_mic96/1024:>7.1f} KB/s   {csv_total_with_mic48/1024:>7.1f} KB/s   基准")
print(f"{'CSV (去datetime)':<20s} {csv_slim_with_mic96/1024:>7.1f} KB/s   {csv_slim_with_mic48/1024:>7.1f} KB/s   {(1-csv_slim_with_mic96/csv_total_with_mic96)*100:>4.1f}%")
print(f"{'BIN':<20s} {bin_total_with_mic96/1024:>7.1f} KB/s   {bin_total_with_mic48/1024:>7.1f} KB/s   {(1-bin_total_with_mic96/csv_total_with_mic96)*100:>4.1f}%")
print()
print(f"卡持续写上限: ~648 KB/s (实测)")
print()
print(f"掉帧情况预测:")
print(f"  CSV+datetime+96kHz: {csv_total_with_mic96/1024:.1f} - 648 = {csv_total_with_mic96/1024 - 648:+.1f} KB/s → 严重掉帧")
print(f"  CSV+datetime+48kHz: {csv_total_with_mic48/1024:.1f} - 648 = {csv_total_with_mic48/1024 - 648:+.1f} KB/s → {'掉帧' if csv_total_with_mic48/1024 > 648 else '安全'}")
print(f"  CSV-datetime+96kHz: {csv_slim_with_mic96/1024:.1f} - 648 = {csv_slim_with_mic96/1024 - 648:+.1f} KB/s → {'掉帧' if csv_slim_with_mic96/1024 > 648 else '安全'}")
print(f"  CSV-datetime+48kHz: {csv_slim_with_mic48/1024:.1f} - 648 = {csv_slim_with_mic48/1024 - 648:+.1f} KB/s → 安全")
print(f"  BIN+96kHz:          {bin_total_with_mic96/1024:.1f} - 648 = {bin_total_with_mic96/1024 - 648:+.1f} KB/s → 安全")
print(f"  BIN+48kHz:          {bin_total_with_mic48/1024:.1f} - 648 = {bin_total_with_mic48/1024 - 648:+.1f} KB/s → 安全")
print()
print("注: 以上计算基于文档中的行/帧大小。需要从实际会话文件验证。")
