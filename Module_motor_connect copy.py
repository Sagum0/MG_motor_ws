#!/usr/bin/env python3
import argparse, serial, sys, time

# ====== 설정 단계 (전역 변수) ======
MOTOR_IDS     = [1, 2]      # 사용할 모터 ID 리스트
DEFAULT_SPEED = 200_000           # 0.01°/s 단위 기본 속도

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

# ====== 4) 명령 하달용 래퍼 함수들 ======
def rotate_multi_loop(ser, motor_id, angle_deg, speed=DEFAULT_SPEED):
    """
    A4: Multi-Loop Angle Control 2
     - angle_deg: 실제 도 단위 소수점 둘째자리
     - speed: 0.01°/s 단위
    """
    a = int(round(angle_deg * 1000))               # 0.01° 단위
    s = int(round(speed))                         # raw 단위
    data = a.to_bytes(8,'little',signed=True) + s.to_bytes(4,'little')
    return send_and_recv_split(ser, 0xA4, motor_id, data)

def rotate_single_loop(ser, motor_id, angle_ctrl, speed=DEFAULT_SPEED, direction='CW'):
    """
    A6: Single-Loop Angle Control 2
     - angle_ctrl: raw unit (1800→180.0°)
     - speed: raw 단위
     - direction: 'CW' or 'CCW'
    """
    spin = 0x00 if direction.upper().startswith('C') else 0x01
    data = bytearray([spin]) \
         + angle_ctrl.to_bytes(2,'little') \
         + b'\x00' \
         + int(speed).to_bytes(4,'little')
    return send_and_recv_split(ser, 0xA6, motor_id, data)

# ====== 5) Idle 상태 → 명령 대기/파싱 ======
def idle_loop(ser):
    PROMPT = ("명령 형식: A4 <ID> <angle_deg> [speed]\n"
              "        A6 <ID> <angle_ctrl> <direction> [speed]\n"
              "        EXIT 입력 시 종료\n")
    print(PROMPT)
    while True:
        line = input(">> ").strip().split()
        if not line: continue
        cmd = line[0].upper()
        if cmd == 'EXIT':
            print("종료합니다."); break
        try:
            mid = int(line[1])
            if mid not in MOTOR_IDS:
                print(f"Unknown motor ID: {mid}"); continue
            if cmd == 'A4':
                angle = float(line[2])
                speed = int(line[3]) if len(line)>3 else DEFAULT_SPEED
                rotate_multi_loop(ser, mid, angle, speed)
            elif cmd == 'A6':
                angle_ctrl = int(line[2])
                direction  = line[3]
                speed      = int(line[4]) if len(line)>4 else DEFAULT_SPEED
                rotate_single_loop(ser, mid, angle_ctrl, speed, direction)
            else:
                print("지원하지 않는 명령입니다.")
        except Exception as e:
            print("명령 파싱 오류:", e)

# ====== 메인 진입점 ======
def main():
    p = argparse.ArgumentParser(description="MG 모터 다중 제어 모듈")
    p.add_argument('-p','--port', default='COM9', help='시리얼 포트')
    p.add_argument('-b','--baud', type=int, default=115200, help='Baud rate')
    args = p.parse_args()

    try:
        ser = init_serial(args.port, args.baud)
        setup_all(ser)
        print(">> Idle 상태 진입, 명령 대기 중...")
        idle_loop(ser)
    except serial.SerialException as e:
        print("포트 오류:", e, file=sys.stderr); sys.exit(1)
    except RuntimeError       as e:
        print("통신 오류:", e, file=sys.stderr); sys.exit(2)
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()
