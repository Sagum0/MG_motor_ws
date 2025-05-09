#!/usr/bin/env python3

import argparse, serial, sys, time

FRAME_HEAD = 0x3E

def calc_checksum(buf: bytearray) -> int:
    return (sum(buf) & 0xFF)

def read_frame(ser: serial.Serial) -> bytearray:
    while True:
        b = ser.read(1)
        if not b: raise RuntimeError("응답 헤더 대기 중 타임아웃")
        if b[0] == FRAME_HEAD: break
    hdr = ser.read(3)
    if len(hdr) < 3: raise RuntimeError("헤더 뒤 바이트 부족")
    ln = hdr[2]
    payload = ser.read(ln + 1)
    if len(payload) < ln + 1: raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv_split(ser, cmd, motor_id, data):
    header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    header_cs = calc_checksum(header)
    data_cs = calc_checksum(data)
    pkt = header + bytes([header_cs]) + data + bytes([data_cs])
    
    ser.reset_input_buffer()
    ser.write(pkt)
    print("TX(ID={motor_id}):"," ".join(f"{b:02X}" for b in pkt))
    time.sleep(0.005)
    
    resp = read_frame(ser)
    print("RX(ID={motor_id}):"," ".join(f"{b:02X}" for b in resp))
    
    return resp

