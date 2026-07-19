#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""验证CSV vs BIN实测产出速率，对比理论值"""
import sys, os, struct, glob

def analyze_session(session_dir):
    """分析会话目录，计算实测产出速率"""

    print("=" * 80)
    print(f"分析会话: {session_dir}")
    print("=" * 80)

    if not os.path.exists(session_dir):
        print(f"错误: 目录不存在")
        return None

    # 1. 获取会话时长 (从MIC.WAV)
    wav_path = os.path.join(session_dir, "MIC.WAV")
    duration_s = None
    mic_rate = None

    if os.path.exists(wav_path):
        wav_size = os.path.getsize(wav_path)
        if wav_size > 44:
            data_size = wav_size - 44
            # 尝试读取WAV头获取采样率
            with open(wav_path, 'rb') as f:
                f.seek(24)
                sample_rate = struct.unpack('<I', f.read(4))[0]
                mic_rate = sample_rate
                duration_s = data_size / (sample_rate * 2)
            print(f"\nMIC.WAV: {wav_size:,} bytes")
            print(f"  采样率: {mic_rate} Hz")
            print(f"  数据大小: {data_size:,} bytes")
            print(f"  时长: {duration_s:.2f}s = {duration_s/60:.2f}分钟")
            print(f"  实测速率: {data_size/duration_s/1024:.1f} KB/s")

    if duration_s is None:
        print("警告: 无法获取会话时长 (MIC.WAV不存在或损坏)")
        return None

    # 2. 分析各数据文件
    print("\n" + "-" * 80)
    print("数据文件分析:")
    print("-" * 80)

    channels = [
        ("ACC_LOW", "加速度低档"),
        ("GYR_LOW", "角速度低档"),
        ("ACC_MID", "加速度中档"),
        ("ACC_HIGH", "加速度高档"),
        ("MAG", "磁力计"),
        ("ENV", "温湿度"),
    ]

    total_data_bytes = 0
    format_type = None

    for prefix, name in channels:
        pattern = os.path.join(session_dir, f"{prefix}*.???")
        files = sorted(glob.glob(pattern))

        if not files:
            continue

        # 检测格式
        ext = os.path.splitext(files[0])[1].upper()
        if format_type is None:
            format_type = ext[1:]  # 去掉点号

        total_size = sum(os.path.getsize(f) for f in files)
        total_data_bytes += total_size
        throughput = total_size / duration_s / 1024

        print(f"  {name:12s} ({prefix:9s}): {len(files):2d}个{ext}文件, "
              f"{total_size:12,} bytes, {throughput:7.1f} KB/s")

    # MIC已经统计过，加到总计中
    mic_bytes = os.path.getsize(wav_path) - 44 if os.path.exists(wav_path) else 0
    total_data_bytes += mic_bytes

    print("-" * 80)
    print(f"  {'总数据量':<25s}: {total_data_bytes:,} bytes")
    print(f"  {'总时长':<25s}: {duration_s:.2f}s = {duration_s/60:.2f}分钟")
    print(f"  {'实测总产出速率':<25s}: {total_data_bytes/duration_s/1024:.1f} KB/s")
    print(f"  {'数据格式':<25s}: {format_type}")

    # 3. 采样CSV/BIN文件的行/帧大小
    print("\n" + "-" * 80)
    print("采样数据格式:")
    print("-" * 80)

    if format_type == "CSV":
        # 采样CSV行长度
        acc_low_files = sorted(glob.glob(os.path.join(session_dir, "ACC_LOW*.CSV")))
        if acc_low_files:
            print(f"\n采样 {os.path.basename(acc_low_files[0])} 前100行:")
            with open(acc_low_files[0], 'rb') as f:
                lines = []
                for i in range(100):
                    line = f.readline()
                    if not line:
                        break
                    lines.append(line)

                if lines:
                    # 跳过表头
                    data_lines = [l for l in lines[1:] if l.strip()]
                    if data_lines:
                        avg_len = sum(len(l) for l in data_lines) / len(data_lines)
                        min_len = min(len(l) for l in data_lines)
                        max_len = max(len(l) for l in data_lines)
                        print(f"  数据行数: {len(data_lines)}")
                        print(f"  平均行长: {avg_len:.1f} bytes")
                        print(f"  范围: {min_len} - {max_len} bytes")

                        # 显示示例行
                        print(f"\n  示例行 (前3行):")
                        for i, line in enumerate(data_lines[:3]):
                            print(f"    {i+1}. {line.decode('utf-8', errors='replace').strip()} ({len(line)}B)")

    elif format_type == "BIN":
        # 采样BIN帧大小
        acc_low_files = sorted(glob.glob(os.path.join(session_dir, "ACC_LOW*.BIN")))
        if acc_low_files:
            print(f"\n采样 {os.path.basename(acc_low_files[0])} 前100帧:")

            # 检测帧大小 (根据通道)
            frame_sizes = {
                "ACC_LOW": 14,
                "GYR_LOW": 14,
                "ACC_MID": 14,
                "ACC_HIGH": 14,
                "MAG": 22,
                "ENV": 24,
            }

            for prefix, _ in channels:
                files = sorted(glob.glob(os.path.join(session_dir, f"{prefix}*.BIN")))
                if not files:
                    continue

                expected_size = frame_sizes.get(prefix, 14)

                with open(files[0], 'rb') as f:
                    data = f.read(expected_size * 10)  # 读10帧
                    if len(data) >= expected_size:
                        print(f"\n  {prefix}: 帧大小 = {expected_size} bytes")
                        # 解析第一帧
                        frame = data[:expected_size]
                        frame_id = struct.unpack('<I', frame[:4])[0]
                        crc = struct.unpack('<I', frame[-4:])[0]
                        print(f"    第1帧: frame_id={frame_id}, CRC32=0x{crc:08X}")

    return {
        "session_dir": session_dir,
        "duration_s": duration_s,
        "total_bytes": total_data_bytes,
        "throughput_kbps": total_data_bytes / duration_s / 1024,
        "format": format_type,
        "mic_rate": mic_rate,
    }

def main():
    if len(sys.argv) < 2:
        print("用法: python verify_throughput.py <session_dir1> [session_dir2] ...")
        print("\n示例:")
        print("  python verify_throughput.py E:/CTBX_2026-07-17-22-26/")
        print("  python verify_throughput.py E:/CTBX_2026-07-18-17-58/")
        print()
        print("或者，直接运行显示理论计算:")
        print("  python calculate_throughput.py")
        sys.exit(1)

    results = []
    for session_dir in sys.argv[1:]:
        result = analyze_session(session_dir)
        if result:
            results.append(result)
        print()

    # 总结对比
    if len(results) >= 2:
        print("\n" + "=" * 80)
        print("会话对比:")
        print("=" * 80)
        print(f"{'会话':<30s} {'格式':<6s} {'mic':<8s} {'时长':<12s} {'实测速率'}")
        print("-" * 80)
        for r in results:
            dirname = os.path.basename(r['session_dir'].rstrip('/\\'))
            print(f"{dirname:<30s} {r['format']:<6s} {r['mic_rate']:>6}Hz "
                  f"{r['duration_s']/60:>6.1f}min   {r['throughput_kbps']:>7.1f} KB/s")

if __name__ == "__main__":
    main()
