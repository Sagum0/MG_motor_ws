#!/usr/bin/env python3
import argparse, serial, sys, time

# ====== 설정 단계 (전역 변수) ======
MOTOR_IDS     = [1, 2]      # 사용할 모터 ID 리스트
DEFAULT_SPEED = 200_000

FRAME_HEAD = 0x3E

# ====== 1) 초기화 단계 ======
def init_serial(port: str, baud: int, timeout: float=0.2) -> serial.Serial:
    """시리얼 포트 열기"""
    return serial.Serial(port, baud, timeout=timeout)

# ====== 2) setup 단계 ======
def connect_motor(ser: serial.Serial, motor_id: int):
    """Ping→productInfo→state3→state2→state1 로 연결 확인"""
    for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
        frame = bytearray([FRAME_HEAD, cmd, motor_id, 0])
        frame.append(calc_checksum(frame))
        ser.reset_input_buffer(); ser.write(frame)
        print(f"TX(ID={motor_id}):", " ".join(f"{b:02X}" for b in frame))
        time.sleep(0.01)
        resp = read_frame(ser)
        print(f"RX(ID={motor_id}):", " ".join(f"{b:02X}" for b in resp))
    print(f">> Motor {motor_id} Connect 완료!")

def setup_all(ser: serial.Serial):
    """모든 모터에 대해 setup 단계 실행"""
    for mid in MOTOR_IDS:
        connect_motor(ser, mid)
    print(">> All motors setup 완료!\n")
    
# ====== 3) 통신에 필요한 공통 함수들 ======
def calc_checksum(buf: bytearray) -> int:
    return sum(buf) & 0xFF

def read_frame(ser: serial.Serial) -> bytearray:
    # 헤더(0x3E) 동기화
    while True:
        b = ser.read(1)
        if not b: raise RuntimeError("응답 헤더 대기 중 타임아웃")
        if b[0] == FRAME_HEAD: break
    hdr = ser.read(3)
    if len(hdr) < 3: raise RuntimeError("헤더 뒤 바이트 부족")
    length = hdr[2]
    payload = ser.read(length+1)
    if len(payload) < length+1: raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv(ser, cmd, motor_id, data=b'') -> bytearray:
    """헤더+데이터 전체 체크섬 방식 (Ping, state 조회 등)"""
    frame = bytearray([FRAME_HEAD, cmd, motor_id, len(data)]) + data
    frame.append(calc_checksum(frame))
    ser.reset_input_buffer(); ser.write(frame)
    print(f"TX(ID={motor_id}):", " ".join(f"{b:02X}" for b in frame))
    time.sleep(0.01)
    resp = read_frame(ser)
    print(f"RX(ID={motor_id}):", " ".join(f"{b:02X}" for b in resp))
    return resp

def send_and_recv_split(ser, cmd, motor_id, data) -> bytearray:
    """
    분리 체크섬 방식 (A4/A6)
     [HEAD,CMD,ID,LEN] + CS(헤더) + DATA + CS(데이터)
    """
    header   = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    header_cs= calc_checksum(header)
    data_cs  = calc_checksum(data)
    pkt      = header + bytes([header_cs]) + data + bytes([data_cs])

    ser.reset_input_buffer(); ser.write(pkt)
    print(f"TX(ID={motor_id}):", " ".join(f"{b:02X}" for b in pkt))
    time.sleep(0.01)
    resp = read_frame(ser)
    print(f"RX(ID={motor_id}):", " ".join(f"{b:02X}" for b in resp))
    return resp