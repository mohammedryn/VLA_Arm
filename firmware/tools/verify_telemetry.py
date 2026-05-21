#!/usr/bin/env python3
"""
Run on RPi5 after flashing the full firmware.
Reads 100 telemetry packets from the ESP32, checks rate and packet integrity.

Usage:
    python3 verify_telemetry.py [/dev/ttyUSB0]
"""
import sys
import time
import serial
import numpy as np

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
BAUD = 2_000_000

TELEMETRY_DTYPE = np.dtype([
    ('timestamp_us',     np.uint32),
    ('servo_pos',        np.float32, (5,)),
    ('servo_load',       np.float32, (5,)),
    ('servo_speed',      np.float32, (5,)),
    ('servo_temp',       np.float32, (5,)),
    ('tof_grid',         np.uint16,  (64,)),
    ('tof_timestamp_us', np.uint32),
    ('tof_resolution',   np.uint8),
    ('tof_valid',        np.uint8),
    ('imu_gyro',         np.float32, (3,)),
    ('imu_accel',        np.float32, (3,)),
    ('contact_flag',     np.uint8),
    ('contact_rms',      np.float32),
    ('safety_clamped',   np.uint8),
    ('checksum',         np.uint16),
])
PACKET_SIZE = 250
assert TELEMETRY_DTYPE.itemsize == PACKET_SIZE, \
    f"dtype size mismatch: {TELEMETRY_DTYPE.itemsize} != {PACKET_SIZE}"

def verify_checksum(raw: bytes) -> bool:
    computed = sum(raw[:-2]) & 0xFFFF
    received = int.from_bytes(raw[-2:], 'little')
    return computed == received

def main():
    print(f"Opening {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=2.0)
    time.sleep(0.5)  # let ESP32 settle
    ser.reset_input_buffer()

    packets = []
    bad_checksums = 0
    t_start = time.monotonic()

    print("Reading 100 packets...")
    while len(packets) < 100:
        raw = ser.read(PACKET_SIZE)
        if len(raw) != PACKET_SIZE:
            print(f"  Short read: got {len(raw)} bytes (timeout?)")
            continue
        if not verify_checksum(raw):
            bad_checksums += 1
            continue
        pkt = np.frombuffer(raw, dtype=TELEMETRY_DTYPE)[0]
        packets.append(pkt)

    elapsed = time.monotonic() - t_start
    rate = len(packets) / elapsed

    print("\n=== Telemetry Verification ===")
    print(f"Packets:        {len(packets)}")
    print(f"Bad checksums:  {bad_checksums}")
    print(f"Elapsed:        {elapsed:.2f}s")
    print(f"Rate:           {rate:.1f} Hz  (target 50 Hz)  "
          f"{'PASS' if 45 < rate < 55 else 'FAIL'}")

    p = packets[-1]
    print(f"\nLatest packet fields:")
    print(f"  timestamp_us:  {p['timestamp_us']}")
    print(f"  servo_pos[5]:  {np.round(p['servo_pos'], 2)}")
    print(f"  servo_temp[5]: {np.round(p['servo_temp'], 1)}")
    print(f"  imu_gyro[3]:   {np.round(p['imu_gyro'], 3)}")
    print(f"  imu_accel[3]:  {np.round(p['imu_accel'], 3)}")
    print(f"  contact_rms:   {p['contact_rms']:.4f}")
    print(f"  tof_valid:     {p['tof_valid']}")

    # Sanity checks
    checks = [
        ("Rate 45–55 Hz",          45 < rate < 55),
        ("No bad checksums",        bad_checksums == 0),
        ("IMU accel non-zero",      np.any(np.abs(p['imu_accel']) > 0.1)),
        ("Servo temps reasonable",  np.all(p['servo_temp'] > 10) and np.all(p['servo_temp'] < 80)),
        ("timestamp_us non-zero",   p['timestamp_us'] > 0),
    ]

    print("\nChecks:")
    all_pass = True
    for label, result in checks:
        status = "PASS" if result else "FAIL"
        print(f"  {status}  {label}")
        if not result:
            all_pass = False

    print(f"\nOverall: {'ALL PASS' if all_pass else 'SOME FAILURES — see above'}")
    ser.close()

if __name__ == '__main__':
    main()
