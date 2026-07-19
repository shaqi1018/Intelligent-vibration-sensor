# 手动验证 LSM 统计

duration_s = 4436
interval_us = 150

expected = duration_s * 1e6 / interval_us
print(f"预期样本数: {expected:,.0f}")

# 三个文件的样本数（从之前的分析）
file1 = 9726821
file2 = 9366267
file3 = 8481048
total = file1 + file2 + file3

print(f"\n实际样本数:")
print(f"  ACC_LOW001.CSV: {file1:,}")
print(f"  ACC_LOW002.CSV: {file2:,}")
print(f"  ACC_LOW003.CSV: {file3:,}")
print(f"  总计: {total:,}")

drop_rate = (1 - total / expected) * 100
print(f"\n掉帧率: {drop_rate:.2f}%")

# 验证时长
actual_duration_s = total * interval_us / 1e6
print(f"\n实际采集时长: {actual_duration_s:.0f}s = {actual_duration_s/60:.1f}min")
print(f"缺失时长: {duration_s - actual_duration_s:.0f}s = {(duration_s - actual_duration_s)/60:.1f}min")

# 交叉验证：检查行数是否与文件大小匹配
print("\n=== 交叉验证 ===")
# LSM ACC_LOW 每行约 43 字节（含 datetime）
avg_line_bytes = 256.1 * 1024 * 1024 / file1
print(f"ACC_LOW001 平均行长: {avg_line_bytes:.1f} 字节")

# 如果三个文件行数差异很大，可能有问题
print(f"\n文件间行数差异:")
print(f"  File1 vs File2: {abs(file1-file2)/file1*100:.1f}%")
print(f"  File2 vs File3: {abs(file2-file3)/file2*100:.1f}%")
