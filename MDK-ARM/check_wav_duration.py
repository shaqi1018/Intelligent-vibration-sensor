import wave
import sys

wav_path = r"E:\CTBX_2026-07-20-01-03\MIC.WAV"

try:
    with wave.open(wav_path, 'rb') as wav_file:
        frames = wav_file.getnframes()
        rate = wav_file.getframerate()
        duration = frames / float(rate)
        print(f"{duration:.3f}")
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)
