#!/usr/bin/env python3
import argparse
import serial
import sys
import time

FRAME_HEAD = 0x3E

def calc_checksum(buf: bytearray) -> int:
    return sum(buf) & 0xFF

def build_frame(cmd: int, motor_id: int, data: bytes = b'') -> bytearray:
    header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    frame = header + data
    frame.append(calc_checksum(frame))
    return frame

def read_frame(ser: serial.Serial) -> bytearray:
    # 헤더(0x3E) 동기화
    while True:
        b = ser.read(1)
        if not b:
            raise RuntimeError("응답 헤더(0x3E) 대기 중 타임아웃")
        if b[0] == FRAME_HEAD:
            break
    hdr = ser.read(3)
    if len(hdr) < 3:
        raise RuntimeError("헤더 뒤 바이트 부족")
    length = hdr[2]
    payload = ser.read(length + 1)
    if len(payload) < length + 1:
        raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv(ser: serial.Serial, cmd: int, motor_id: int, data: bytes = b''):
    frame = build_frame(cmd, motor_id, data)
    ser.reset_input_buffer()
    ser.write(frame)
    print("TX:", " ".join(f"{b:02X}" for b in frame))
    time.sleep(0.01)
    resp = read_frame(ser)
    print("RX:", " ".join(f"{b:02X}" for b in resp))
    return resp

def send_and_recv_a4(ser: serial.Serial, motor_id: int, data: bytes):
    HEADER_CMD = 0xA4
    header = bytearray([FRAME_HEAD, HEADER_CMD, motor_id, len(data)])
    header_cs = calc_checksum(header)
    data_cs   = calc_checksum(data)
    packet = header + bytearray([header_cs]) + data + bytearray([data_cs])

    ser.reset_input_buffer()
    ser.write(packet)
    print("TX:", " ".join(f"{b:02X}" for b in packet))
    time.sleep(0.01)
    resp = read_frame(ser)
    print("RX:", " ".join(f"{b:02X}" for b in resp))
    return resp

def send_and_recv_a6(ser: serial.Serial, motor_id: int, spin_dir: int, angle_ctrl: int, max_speed: int) -> bytearray:
    HEADER_CMD = 0xA6
    # 1) 데이터 구성: [spin_dir(1)] + [angle_ctrl(2)] + [0x00(1)] + [max_speed(4)]
    data = bytearray([spin_dir]) \
         + angle_ctrl.to_bytes(2, 'little') \
         + b'\x00' \
         + max_speed.to_bytes(4, 'little')

    # 2) 헤더 구성 및 헤더 체크섬
    header = bytearray([FRAME_HEAD, HEADER_CMD, motor_id, len(data)])
    header_cs = calc_checksum(header)

    # 3) 데이터 체크섬
    data_cs = calc_checksum(data)

    # 4) 패킷: [헤더][헤더CS][데이터][데이터CS]
    packet = header + bytes([header_cs]) + data + bytes([data_cs])

    # 5) 전송/수신
    ser.reset_input_buffer()
    ser.write(packet)
    print("TX:", " ".join(f"{b:02X}" for b in packet))

    time.sleep(0.01)
    resp = read_frame(ser)
    print("RX:", " ".join(f"{b:02X}" for b in resp))

    return resp


def connect(ser: serial.Serial, motor_id: int):
    send_and_recv(ser, cmd=0x1F, motor_id=motor_id)  # Ping
    send_and_recv(ser, cmd=0x12, motor_id=motor_id)  # productInfo
    send_and_recv(ser, cmd=0x16, motor_id=motor_id)  # state3
    send_and_recv(ser, cmd=0x14, motor_id=motor_id)  # state2
    send_and_recv(ser, cmd=0x10, motor_id=motor_id)  # state1
    print(">> Connect 완료!\n")

def main():
    p = argparse.ArgumentParser(description="인터랙티브 MG 모터 회전 예제")
    p.add_argument('-p','--port', default='COM9', help='시리얼 포트')
    p.add_argument('-b','--baud', type=int, default=115200, help='Baud rate')
    p.add_argument('-i','--id',   type=int, default=1, help='모터 ID')
    args = p.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            connect(ser, args.id)
            while 1:

                # 사용자 입력
                angle = float(input("회전할 각도 입력 (도 단위, 소수점 둘째자리까지): "))
                # 내부값 = 각도 × 1000
                val = int(round(angle * 1000))
                print(f"내부 전송 값: {val}")

                # little-endian 8바이트 + 속도 4바이트
                data = val.to_bytes(8, 'little', signed=True) + (400_000).to_bytes(4, 'little')
                send_and_recv_a4(ser, args.id, data=data)

    except serial.SerialException as e:
        print(f"포트 열기 실패: {e}", file=sys.stderr)
        sys.exit(1)
    except RuntimeError as e:
        print(f"통신 오류: {e}", file=sys.stderr)
        sys.exit(2)

if __name__ == '__main__':
    main()
