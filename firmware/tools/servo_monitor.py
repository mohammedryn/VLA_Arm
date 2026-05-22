#!/usr/bin/env python3
"""
Live servo position monitor.
Move the arm physically and watch joint angles update in real-time.
Usage: python3 servo_monitor.py [/dev/ttyUSB1]
"""
import sys, struct, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB1'
BAUD = 2_000_000
MAGIC = bytes([0x5a, 0xa5, 0x5a, 0xa5])


def find_packet(s):
    tail = bytearray()
    while True:
        b = s.read(1)
        if not b:
            return None
        tail += b
        if len(tail) >= 4 and bytes(tail[-4:]) == MAGIC:
            rest = s.read(250)
            if len(rest) == 250:
                return bytes(tail[-4:]) + rest
            tail = tail[-3:]


def main():
    print(f"Connecting to {PORT} at {BAUD} baud...")
    s = serial.Serial(PORT, BAUD, timeout=2)
    time.sleep(0.3)
    s.reset_input_buffer()

    print("Syncing...")
    pkt = find_packet(s)
    if pkt is None:
        print("No packets — check firmware and port.")
        return
    print("Synced. Move the arm. Ctrl+C to stop.\n")

    # Header
    print(f"{'J0-Base':>10} {'J1a-Shldr':>10} {'J1b-Shldr':>10} {'J2-Elbow':>10} {'J3-Grip':>10}  {'Temp(C)':>18}")
    print("-" * 80)

    try:
        while True:
            pkt = find_packet(s)
            if pkt is None:
                continue
            # Packet layout (254 bytes):
            #   [0:4]   magic
            #   [4:8]   timestamp_us
            #   [8:28]  servo_pos[5]   ← floats
            #   [28:48] servo_load[5]
            #   [48:68] servo_speed[5]
            #   [68:88] servo_temp[5]  ← floats
            pos  = struct.unpack_from('<5f', pkt, 8)
            temp = struct.unpack_from('<5f', pkt, 68)

            pos_str  = '  '.join(f'{p:+7.1f}' for p in pos)
            temp_str = '  '.join(f'{t:4.1f}' for t in temp)
            print(f"{pos_str}    {temp_str}", end='\r')
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        s.close()


if __name__ == '__main__':
    main()
