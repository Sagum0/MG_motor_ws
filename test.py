#!/usr/bin/env python3
import argparse, serial, sys, time

FRAME_HEAD = 0x3E

def calc_checksum(buf: bytearray) -> int:
    """하위 8비트 체크섬 (sum(buf)&0xFF)"""
    return sum(buf) & 0xFF

def read_frame(ser: serial.Serial) -> bytearray:
    """0x3E 헤더로 동기화한 뒤, [HEAD,CMD,ID,LEN] + DATA + CS 읽기"""
    # 1) 헤더(0x3E) 기다리기
    while True:
        b = ser.read(1)
        if not b: raise RuntimeError("응답 헤더 대기 중 타임아웃")
        if b[0] == FRAME_HEAD: break
    # 2) CMD, ID, LEN
    hdr = ser.read(3)
    if len(hdr) < 3: raise RuntimeError("헤더 뒤 바이트 부족")
    length = hdr[2]
    # 3) 데이터 + 체크섬
    payload = ser.read(length + 1)
    if len(payload) < length+1: raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv(ser: serial.Serial, cmd: int, motor_id: int, data: bytes=b'') -> bytearray:
    """
    기본 명령 전송/수신:
     - [HEAD,CMD,ID,LEN] + DATA + CS(전체)
    """
    frame = bytearray([FRAME_HEAD, cmd, motor_id, len(data)]) + data
    frame.append(calc_checksum(frame))
    ser.reset_input_buffer(); ser.write(frame)
    print("TX:", " ".join(f"{b:02X}" for b in frame))
    time.sleep(0.01)
    resp = read_frame(ser)
    print("RX:", " ".join(f"{b:02X}" for b in resp))
    return resp

def send_and_recv_split(ser: serial.Serial, cmd: int, motor_id: int, data: bytes) -> bytearray:
    """
    A4/A6 전용 전송/수신:
     - [HEAD,CMD,ID,LEN] + CS(헤더) + DATA + CS(데이터)
    """
    header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    header_cs = calc_checksum(header)
    data_cs   = calc_checksum(data)
    packet = header + bytes([header_cs]) + data + bytes([data_cs])

    ser.reset_input_buffer(); ser.write(packet)
    print("TX:", " ".join(f"{b:02X}" for b in packet))
    time.sleep(0.01)
    resp = read_frame(ser)
    print("RX:", " ".join(f"{b:02X}" for b in resp))
    return resp

def connect(ser: serial.Serial, motor_id: int):
    """Ping→productInfo→state3→state2→state1 으로 연결 확인"""
    for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
        send_and_recv(ser, cmd, motor_id)
    print(">> Connect 완료!\n")

def rotate_multi_loop(ser: serial.Serial, motor_id: int,
                      angle_deg: float, speed_dps: float = 2000.0):
    """
    Multi-Loop Angle Control 2 (0xA4)
     - angle_deg: 목표 각도(°) → 내부 0.01°/LSB 단위
     - speed_dps: 목표 속도(°/s) → 내부 0.01dps/LSB 단위
    """
    angle_lsb = int(round(angle_deg * 100))            # 0.01° 단위
    speed_lsb = int(round(speed_dps * 100))            # 0.01°/s 단위
    data = angle_lsb.to_bytes(8, 'little', signed=True) \
         + speed_lsb.to_bytes(4, 'little')
    return send_and_recv_split(ser, 0xA4, motor_id, data)

def rotate_single_loop(ser: serial.Serial, motor_id: int,
                       angle_ctrl: int,    # 0~3599 (0.1° 단위), ex. 1800 → 180°
                       speed_lsb: int = 200_000,
                       direction: str = 'CW'):
    """
    Single-Loop Angle Control 2 (0xA6) 래퍼
     - angle_ctrl: 0.1°/LSB 단위 raw value (ex. 1800 → 180°) :contentReference[oaicite:0]{index=0}:contentReference[oaicite:1]{index=1}
     - speed_lsb: 0.01dps/LSB 단위 (기본 200000)
     - direction: 'CW' 또는 'CCW'
    """
    # 1) spinDirection
    spin = 0x00 if direction.upper().startswith('C') else 0x01
    # 2) payload: spin(1) + angle(2) + 0x00(1) + speed(4)
    data = bytearray([spin]) \
         + angle_ctrl.to_bytes(2, 'little') \
         + b'\x00' \
         + speed_lsb.to_bytes(4, 'little')
    return send_and_recv_split(ser, 0xA6, motor_id, data)

def main():
    p = argparse.ArgumentParser(description="MG 모터 RS-485 제어 예제")
    p.add_argument('-p','--port', default='COM9', help='시리얼 포트')
    p.add_argument('-b','--baud', type=int, default=115200, help='Baud rate')
    p.add_argument('-i','--id',   type=int, default=1, help='모터 ID')
    args = p.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            connect(ser, args.id)

            # --- Multi-Loop 예시 (180° 회전) ---
            rotate_multi_loop(ser, args.id, angle_deg=1800.0)
            time.sleep(3)

            # --- Single-Loop 예시 (180° 회전) ---
            # angle_ctrl=1800 → 180.0° (0.1°/LSB) 
            rotate_single_loop(ser, args.id, angle_ctrl=1800, direction='CCW')

    except serial.SerialException as e:
        print("포트 오류:", e, file=sys.stderr); sys.exit(1)
    except RuntimeError as e:
        print("통신 오류:", e, file=sys.stderr); sys.exit(2)

if __name__ == '__main__':
    main()
