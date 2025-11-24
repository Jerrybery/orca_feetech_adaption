#!/usr/bin/env python3
"""
Set an explicit target position (counts) and attempt move with torque+speed.

Usage:
  python scripts/set_target_position.py --com COM5 --baud 1000000 --id 1 --target 2000 --speed 800

This script:
 - reads current pos (addr 0x38)
 - enables torque (addr 0x28 <- 1)
 - sets speed (addr 0x20 <- speed)
 - writes target (addr 0x2A <- target counts low,high)
 - reads back position
"""
import serial, time, argparse

def calc_checksum(bytes_seq):
    return (~(sum(bytes_seq) & 0xFF)) & 0xFF

def build_write_packet(servo_id: int, address: int, data_bytes: bytes):
    params = [address & 0xFF] + list(data_bytes)
    length_field = len(params) + 2
    pkt = bytearray([0xFF, 0xFF, servo_id & 0xFF, length_field & 0xFF, 0x03]) + bytearray(params)
    pkt.append(calc_checksum(pkt[2:]))
    return pkt

def build_read_packet(servo_id: int, address: int, length: int):
    params = [address & 0xFF, length & 0xFF]
    length_field = len(params) + 2
    pkt = bytearray([0xFF,0xFF,servo_id & 0xFF,length_field & 0xFF,0x02]) + bytearray(params)
    pkt.append(calc_checksum(pkt[2:]))
    return pkt

def hexdump(b): return ' '.join(f"{x:02X}" for x in b)

def parse_position_response(resp):
    if len(resp) < 6: return None
    if resp[0] != 0xFF or resp[1] != 0xFF: return None
    length = resp[3]
    data = resp[5:5+(length-2)]
    if len(data) >= 2:
        counts = data[0] | (data[1] << 8)
        return counts
    return None

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--com', required=True)
    p.add_argument('--baud', type=int, default=1000000)
    p.add_argument('--id', type=int, default=1)
    p.add_argument('--target', type=int, required=True, help='explicit target counts (0..4095)')
    p.add_argument('--speed', type=int, default=400)
    p.add_argument('--timeout', type=float, default=0.12)
    args = p.parse_args()

    ser = serial.Serial(port=args.com, baudrate=args.baud, bytesize=8, parity=serial.PARITY_NONE, stopbits=1, timeout=0.1)

    sid = args.id
    # read current pos
    rpk = build_read_packet(sid, 0x38, 2)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(rpk); ser.flush()
    time.sleep(args.timeout)
    resp = ser.read(ser.in_waiting or 128)
    print("Initial READ RAW:", hexdump(resp))
    counts = parse_position_response(resp)
    if counts is None:
        print("Failed to read current pos. Aborting.")
        ser.close(); return
    print("Current counts:", counts)

    # enable torque (address 0x28, write 1)
    print("Enabling torque...")
    en_pkt = build_write_packet(sid, 0x28, bytes([0x01]))
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(en_pkt); ser.flush()
    time.sleep(0.05)
    print("EN WRITE RAW:", hexdump(ser.read(ser.in_waiting or 64)))

    # set speed (address 0x20, 2 bytes little endian)
    sp = max(1, min(65535, int(args.speed)))
    low = sp & 0xFF; high = (sp >> 8) & 0xFF
    print(f"Setting speed {sp}...")
    sp_pkt = build_write_packet(sid, 0x20, bytes([low, high]))
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(sp_pkt); ser.flush()
    time.sleep(0.05)
    print("SPEED WRITE RAW:", hexdump(ser.read(ser.in_waiting or 64)))

    # write explicit target position
    tgt = int(args.target) & 0x0FFF
    lc = tgt & 0xFF; hc = (tgt >> 8) & 0xFF
    print("Writing explicit target counts:", tgt)
    wpk = build_write_packet(sid, 0x2A, bytes([lc, hc]))
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(wpk); ser.flush()
    time.sleep(0.1)
    print("WRITE RESP RAW:", hexdump(ser.read(ser.in_waiting or 128)))

    # read back after a short wait
    time.sleep(0.4)
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(rpk); ser.flush()
    time.sleep(args.timeout)
    rb = ser.read(ser.in_waiting or 128)
    print("READ BACK RAW:", hexdump(rb))
    rb_counts = parse_position_response(rb)
    print("Read back counts:", rb_counts)
    ser.close()

if __name__ == '__main__':
    main()