# Configuration from CONFIG.JSN
duration_s = 4436
qma_odr_hz = 1600
h3_odr_hz = 400
mag_odr_hz = 100

# Actual data line counts (excluding headers and padding)
qma_actual = 6305841
h3_actual = 1582469
mag_actual = 403201

# Calculate expected samples
qma_expected = duration_s * qma_odr_hz
h3_expected = duration_s * h3_odr_hz
mag_expected = duration_s * mag_odr_hz

# Calculate drop rates
qma_drop_rate = (1 - qma_actual / qma_expected) * 100
h3_drop_rate = (1 - h3_actual / h3_expected) * 100
mag_drop_rate = (1 - mag_actual / mag_expected) * 100

print("=" * 60)
print("传感器掉帧率分析报告")
print("=" * 60)
print(f"\n会话时长: {duration_s} 秒")
print("\n" + "-" * 60)
print("QMA (ACC_MID):")
print(f"  配置采样率: {qma_odr_hz} Hz")
print(f"  预期样本数: {qma_expected:,}")
print(f"  实际样本数: {qma_actual:,}")
print(f"  丢失样本数: {qma_expected - qma_actual:,}")
print(f"  掉帧率: {qma_drop_rate:.1f}%")

print("\n" + "-" * 60)
print("H3 (ACC_HIGH):")
print(f"  配置采样率: {h3_odr_hz} Hz")
print(f"  预期样本数: {h3_expected:,}")
print(f"  实际样本数: {h3_actual:,}")
print(f"  丢失样本数: {h3_expected - h3_actual:,}")
print(f"  掉帧率: {h3_drop_rate:.1f}%")

print("\n" + "-" * 60)
print("MAG (磁力计):")
print(f"  配置采样率: {mag_odr_hz} Hz")
print(f"  预期样本数: {mag_expected:,}")
print(f"  实际样本数: {mag_actual:,}")
print(f"  丢失样本数: {mag_expected - mag_actual:,}")
print(f"  掉帧率: {mag_drop_rate:.1f}%")

print("\n" + "=" * 60)
print("总结:")
print(f"  QMA:  {qma_drop_rate:.1f}%")
print(f"  H3:   {h3_drop_rate:.1f}%")
print(f"  MAG:  {mag_drop_rate:.1f}%")
print("=" * 60)
