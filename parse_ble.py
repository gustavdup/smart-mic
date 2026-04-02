#!/usr/bin/env python3
"""
parse_ble.py  —  extract and print the BLE snapshot log embedded in a recorder WAV file.

Usage:
    python3 parse_ble.py path/to/rec_20260330_144406.wav
"""

import os
import struct
import sys

# Snapshot layout (packed, matches firmware #pragma pack(push,1)):
#   uint32  offset_ms
#   × 5 beacons:
#       uint8   active   (bool)
#       int8    rssi
#       float   distance (4 bytes, little-endian)
#
# = 4 + 5 × 6 = 34 bytes per snapshot

SNAPSHOT_SIZE = 4 + 5 * (1 + 1 + 4)   # 34
NUM_BEACONS   = 5


OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ble_logs")


def parse(path: str):
    with open(path, "rb") as f:
        data = f.read()

    idx = data.find(b"ble_")
    if idx == -1:
        print("No BLE chunk found in this file.")
        return

    (chunk_size,) = struct.unpack_from("<I", data, idx + 4)
    payload = data[idx + 8 : idx + 8 + chunk_size]
    count = chunk_size // SNAPSHOT_SIZE

    lines = []
    lines.append(f"File   : {path}")
    lines.append(f"Snapshots: {count}  ({count}s of BLE data)\n")

    header = f"{'Time':>7s}  " + "  ".join(f"{'T'+str(i+1):>18s}" for i in range(NUM_BEACONS))
    lines.append(header)
    lines.append("-" * len(header))

    for i in range(count):
        base = i * SNAPSHOT_SIZE
        (offset_ms,) = struct.unpack_from("<I", payload, base)
        cols = []
        pos = base + 4
        for _ in range(NUM_BEACONS):
            active = struct.unpack_from("B", payload, pos)[0]
            rssi   = struct.unpack_from("b", payload, pos + 1)[0]
            (dist,) = struct.unpack_from("<f", payload, pos + 2)
            pos += 6
            cols.append(f"{rssi:+d}dBm {dist:.1f}m" if active else "--")

        lines.append(f"{offset_ms/1000:>6.1f}s  " + "  ".join(f"{c:>18s}" for c in cols))

    output = "\n".join(lines)
    print(output)

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    stem = os.path.splitext(os.path.basename(path))[0]
    out_path = os.path.join(OUTPUT_DIR, stem + ".txt")
    with open(out_path, "w") as f:
        f.write(output + "\n")
    print(f"\nSaved → {out_path}")


if __name__ == "__main__":
    if len(sys.argv) >= 2:
        path = sys.argv[1]
    else:
        import subprocess
        result = subprocess.run(
            ["osascript", "-e",
             'POSIX path of (choose file with prompt "Select a recorded WAV file" of type {"wav", "WAV"})'],
            capture_output=True, text=True
        )
        path = result.stdout.strip()
        if not path:
            print("No file selected.")
            sys.exit(0)
    parse(path)
