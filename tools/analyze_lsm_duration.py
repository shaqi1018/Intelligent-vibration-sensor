# LSM 实际采集了多久？
duration_s = 4436  # CONFIG.JSN 记录的总时长
interval_us = 150  # LSM间隔

expected = duration_s * 1e6 / interval_us
actual = 9726821

print(f"Duration: {duration_s}s = {duration_s/60:.1f}min")
print(f"Expected (74min): {expected:,.0f}")
print(f"Actual samples: {actual:,}")
print(f"Drop rate: {(1 - actual/expected)*100:.1f}%")
print()

# 但实际上LSM只采集了多久？
actual_duration = actual * interval_us / 1e6
print(f"Actual LSM duration: {actual_duration:.0f}s = {actual_duration/60:.1f}min")
print(f"Missing time: {duration_s - actual_duration:.0f}s = {(duration_s - actual_duration)/60:.1f}min")
print()

# Ring状态正常（hwm=98303/98304，drop=0），说明数据产出正常
# 问题是：为什么LSM只产出了24.3分钟的数据？
print("结论：LSM 任务可能在采集中途被阻塞或停止了")
