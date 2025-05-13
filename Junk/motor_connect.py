#!/usr/bin/env python3
import argparse
import serial
import sys
import time

FRAME_HEAD = 0x3E

def calc_checksum(buf: bytearray) -> int:
    """CMD_SUM: frame[0]~frame[3] 합의 하위 8비트"""
    return sum(buf) & 0xFF

def build_frame(cmd: int, motor_id: int, data: bytes = b'') -> bytearray:
    frame = bytearray([FRAME_HEAD, cmd, motor_id, len(data)]) + data
    frame.append(calc_checksum(frame))
    return frame

def read_frame(ser: serial.Serial) -> bytearray:
    """0x3E 헤더로 싱크 맞춘 뒤 전체 프레임 읽기"""
    # 1) 헤더(0x3E) 대기
    while True:
        b = ser.read(1)
        if not b:
            raise RuntimeError("응답 헤더(0x3E) 대기 중 타임아웃")
        if b[0] == FRAME_HEAD:
            break
    # 2) cmd, id, length 읽기
    hdr = ser.read(3)
    if len(hdr) < 3:
        raise RuntimeError("헤더 뒤 바이트 부족")
    length = hdr[2]
    # 3) data + checksum 읽기
    payload = ser.read(length + 1)
    if len(payload) < length + 1:
        raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv_no_echo(ser: serial.Serial, cmd: int, motor_id: int):
    frame = build_frame(cmd, motor_id)
    ser.reset_input_buffer()    # 이전 잔여 데이터 전부 버림
    ser.write(frame)            # TX
    print(f"TX: {' '.join(f'{b:02X}' for b in frame)}")

    # → Non-echo 모드라면 TX 데이터가 곧바로 버려지고, 곧바로 모터 응답이 온다
    time.sleep(0.01)
    resp = read_frame(ser)
    print(f"RX: {' '.join(f'{b:02X}' for b in resp)}")
    return resp

def connect(ser: serial.Serial, motor_id: int):
    send_and_recv_no_echo(ser, cmd=0x1F, motor_id=motor_id)  # Ping
    send_and_recv_no_echo(ser, cmd=0x12, motor_id=motor_id)  # productInfo
    send_and_recv_no_echo(ser, cmd=0x16, motor_id=motor_id)  # state3
    send_and_recv_no_echo(ser, cmd=0x14, motor_id=motor_id)  # state2
    send_and_recv_no_echo(ser, cmd=0x10, motor_id=motor_id)  # state1

def main():
    p = argparse.ArgumentParser(description="MG 모터 RS-485 Connect (Non-Echo 모드)")
    p.add_argument('-p','--port', default='COM9', help='시리얼 포트')
    p.add_argument('-b','--baud', type=int, default=115200, help='Baud rate')
    p.add_argument('-i','--id',   type=int, default=1, help='모터 ID')
    args = p.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            connect(ser, args.id)
    except serial.SerialException as e:
        print(f"포트 열기 실패: {e}", file=sys.stderr)
        sys.exit(1)
    except RuntimeError as e:
        print(f"통신 오류: {e}", file=sys.stderr)
        sys.exit(2)

if __name__ == '__main__':
    main()
