#!/usr/bin/env python3
"""
Analyze LSM sensor drop frame rate from CSV files.
"""

import json
import sys
from pathlib import Path

def analyze_session(session_dir):
    session_path = Path(session_dir)

    # Read CONFIG.JSN
    config_path = session_path / "CONFIG.JSN"
    if not config_path.exists():
        print(f"ERROR: CONFIG.JSN not found in {session_dir}")
        return None

    with open(config_path, 'r', encoding='utf-8') as f:
        config = json.load(f)

    # Extract session parameters
    duration_s = config['timebase']['duration_s']
    odr_hz = config['accel_low']['odr_hz']

    # Calculate expected samples
    expected_samples = duration_s * odr_hz

    print(f"Session Directory: {session_dir}")
    print(f"Duration: {duration_s} seconds")
    print(f"ODR: {odr_hz} Hz")
    print(f"Expected samples per sensor: {expected_samples:,}")
    print()

    # Count actual samples from CSV files
    results = {}

    for sensor_name, file_pattern in [('ACC_LOW', 'ACC_LOW*.CSV'), ('GYR_LOW', 'GYR_LOW*.CSV')]:
        total_samples = 0
        files = sorted(session_path.glob(file_pattern))

        if not files:
            print(f"WARNING: No {file_pattern} files found")
            continue

        for csv_file in files:
            with open(csv_file, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                # Skip header (line 1) and padding line (line 2 with #)
                data_lines = [l for l in lines[2:] if l.strip() and not l.startswith('#')]
                total_samples += len(data_lines)

        results[sensor_name] = {
            'files': len(files),
            'actual_samples': total_samples
        }

        # Calculate drop rate
        drop_rate = (1 - total_samples / expected_samples) * 100

        print(f"{sensor_name}:")
        print(f"  Files: {len(files)}")
        print(f"  Actual samples: {total_samples:,}")
        print(f"  Drop rate: {drop_rate:.2f}%")
        print()

    # Check for BIN files (which contain CRC32)
    bin_files = list(session_path.glob("*.BIN"))
    if bin_files:
        print("Note: BIN files found. CRC32 validation available in BIN format.")
    else:
        print("Note: CSV format only (no BIN files). CRC32 validation not applicable for CSV.")

    return {
        'duration_s': duration_s,
        'odr_hz': odr_hz,
        'expected_samples': expected_samples,
        'results': results
    }

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python analyze_lsm_dropframe.py <session_directory>")
        sys.exit(1)

    session_dir = sys.argv[1]
    analyze_session(session_dir)
