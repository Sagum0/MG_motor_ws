#!/usr/bin/env python3
import argparse
import serial
import sys
import time

FRAME_HEAD = 0x3E

def calc_checksum(buf: bytearray) -> int:
    """하위 8비트 체크섬 (sum(buf) & 0xFF)"""
    return sum(buf) & 0xFF

def build_frame(cmd: int, motor_id: int, data: bytes = b'') -> bytearray:
    """
    헤더만 있는 기본 명령 프레임 생성:
      [HEAD, CMD, ID, LEN] + DATA + [CHECKSUM]
    """
    header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    frame = header + data
    frame.append(calc_checksum(frame))
    return frame

def read_frame(ser: serial.Serial) -> bytearray:
    """0x3E 헤더로 싱크 후 전체 프레임(HEAD+CMD+ID+LEN+DATA+CS) 읽기"""
    # 1) 헤더 찾기
    while True:
        b = ser.read(1)
        if not b:
            raise RuntimeError("응답 헤더(0x3E) 대기 중 타임아웃")
        if b[0] == FRAME_HEAD:
            break
    # 2) CMD, ID, LEN 읽기
    hdr = ser.read(3)
    if len(hdr) < 3:
        raise RuntimeError("헤더 뒤 바이트 부족")
    length = hdr[2]
    # 3) DATA + CS 읽기
    payload = ser.read(length + 1)
    if len(payload) < length + 1:
        raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv(ser: serial.Serial, cmd: int, motor_id: int, data: bytes = b'') -> bytearray:
    """
    기본 명령 전송/수신:
      - build_frame 사용
      - ser.reset_input_buffer()로 잔여 데이터 삭제
      - TX, RX 로그 출력
    """
    frame = build_frame(cmd, motor_id, data)
    ser.reset_input_buffer()
    ser.write(frame)
    print("TX:", " ".join(f"{b:02X}" for b in frame))
    # 짧게 대기 후 수신
    time.sleep(0.01)
    resp = read_frame(ser)
    print("RX:", " ".join(f"{b:02X}" for b in resp))
    return resp

def send_and_recv_a4(ser: serial.Serial, motor_id: int, data: bytes) -> bytearray:
    """
    Multi-Loop Angle Control 전용 전송/수신:
      1) 헤더 = [HEAD, 0xA4, ID, len(data)]
      2) 헤더 CS = sum(HEAD~LEN)&0xFF
      3) DATA (8바이트 angle + 4바이트 speed)
      4) DATA CS = sum(DATA)&0xFF
      => [헤더][헤더CS][DATA][데이터CS]
    """
    HEADER_CMD = 0xA4
    header = bytearray([FRAME_HEAD, HEADER_CMD, motor_id, len(data)])
    header_cs = calc_checksum(header)
    data_cs = calc_checksum(data)
    packet = header + bytearray([header_cs]) + data + bytearray([data_cs])

    ser.reset_input_buffer()
    ser.write(packet)
    print("TX:", " ".join(f"{b:02X}" for b in packet))

    time.sleep(0.01)
    resp = read_frame(ser)
    print("RX:", " ".join(f"{b:02X}" for b in resp))
    return resp

def connect(ser: serial.Serial, motor_id: int):
    """Ping, productInfo, state3, state2, state1 순으로 기본 연결 확인"""
    send_and_recv(ser, cmd=0x1F, motor_id=motor_id)  # Ping
    send_and_recv(ser, cmd=0x12, motor_id=motor_id)  # Read productInfo
    send_and_recv(ser, cmd=0x16, motor_id=motor_id)  # Read state3
    send_and_recv(ser, cmd=0x14, motor_id=motor_id)  # Read state2
    send_and_recv(ser, cmd=0x10, motor_id=motor_id)  # Read state1
    print(">> Connect 완료!\n")

def main():
    parser = argparse.ArgumentParser(description="MG 모터 RS-485 제어 (Non-Echo 모드)")
    parser.add_argument('-p','--port', default='COM9', help='시리얼 포트')
    parser.add_argument('-b','--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('-i','--id',   type=int, default=1, help='모터 ID')
    args = parser.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            # 1) Connect
            connect(ser, args.id)

            # 2) 0° 회전 (Multi-Loop Angle Control 2)
            #    angle=0 ⇒ little-endian 8바이트 / speed=200000 ⇒ 4바이트
            zero_angle = (0).to_bytes(8, 'little')
            speed     = (400000).to_bytes(4, 'little')
            send_and_recv_a4(ser, args.id, data=zero_angle + speed)

            # 3초 대기
            time.sleep(1)

            # 3) 180° 회전
            oneeighty = (180000).to_bytes(8, 'little')  # 180.00° → 18000 (0.01° 단위 가정)
            send_and_recv_a4(ser, args.id, data=oneeighty + speed)

    except serial.SerialException as e:
        print(f"포트 열기 실패: {e}", file=sys.stderr)
        sys.exit(1)
    except RuntimeError as e:
        print(f"통신 오류: {e}", file=sys.stderr)
        sys.exit(2)

if __name__ == '__main__':
    main()
