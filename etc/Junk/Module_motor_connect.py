#!/usr/bin/env python3
import argparse, serial, sys, time, threading

# ====== 0) 설정 (전역) ======
MOTOR_IDS     = [1, 2]       # 사용할 모터 ID 리스트
POLL_HZ       = 100          # 폴링 주파수 (Hz)
DEFAULT_SPEED = 200_000      # 0.01°/s 단위 기본 속도
FRAME_HEAD    = 0x3E

ser_lock = threading.Lock()  # 시리얼 버스 동기화 락

# ====== 1) 시리얼 초기화 ======
def init_serial(port: str, baud: int, timeout: float = 0.1) -> serial.Serial:
    """시리얼 포트 열기 & RTS-as-DE 설정"""
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
    except serial.SerialException as e:
        print(f"포트 열기 실패 '{port}': {e}", file=sys.stderr)
        sys.exit(1)
    ser.setRTS(False)  # DE/RE 토글: 초기엔 수신 모드
    print(f">> Opened {port} @ {baud}bps (RTS-as-DE)")
    return ser

# ====== 2) 공통 유틸 함수 ======
def calc_checksum(buf: bytearray) -> int:
    return sum(buf) & 0xFF

def read_frame(ser: serial.Serial) -> bytearray:
    """0x3E 헤더 싱크 → [cmd,id,len] → data+cs 읽기"""
    while True:
        b = ser.read(1)
        if not b:
            raise RuntimeError("응답 헤더 대기 중 타임아웃")
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

# ====== 3) 전송 함수 (락 적용) ======
def send_basic(ser: serial.Serial, cmd: int, mid: int, data: bytes = b'') -> bytearray:
    """
    [HEAD,CMD,ID,LEN]+DATA+CS(전체) 프로토콜
    DE/RE 제어 & 락으로 완전 동기화
    """
    frame = bytearray([FRAME_HEAD, cmd, mid, len(data)]) + data
    frame.append(calc_checksum(frame))

    with ser_lock:
        ser.reset_input_buffer()
        ser.setRTS(True)
        ser.write(frame)
        ser.flush()
        ser.setRTS(False)

        print(f"TX(ID={mid}):", " ".join(f"{b:02X}" for b in frame))
        resp = read_frame(ser)
        print(f"RX(ID={mid}):", " ".join(f"{b:02X}" for b in resp))
    return resp

def send_split(ser: serial.Serial, cmd: int, mid: int, data: bytes) -> bytearray:
    """
    [HEAD,CMD,ID,LEN]+CS(header)+DATA+CS(data) (A6 전용)
    락 & DE/RE 제어 포함
    """
    header = bytearray([FRAME_HEAD, cmd, mid, len(data)])
    hdr_cs = calc_checksum(header)
    data_cs = calc_checksum(data)
    packet = header + bytes([hdr_cs]) + data + bytes([data_cs])

    with ser_lock:
        ser.reset_input_buffer()
        ser.setRTS(True)
        ser.write(packet)
        ser.flush()
        ser.setRTS(False)

        print(f"TX(ID={mid}):", " ".join(f"{b:02X}" for b in packet))
        resp = read_frame(ser)
        print(f"RX(ID={mid}):", " ".join(f"{b:02X}" for b in resp))
    return resp

# ====== 4) Connect / Zero / Disconnect ======
def connect_all(ser: serial.Serial):
    """0x1F→0x12→0x16→0x14→0x10 순으로 모터 연결 확인"""
    for mid in MOTOR_IDS:
        for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
            try:
                send_basic(ser, cmd, mid)
            except Exception as e:
                print(f"[Warn] Connect(ID={mid},0x{cmd:02X}) failed:", e)
    print(">> All motors connected.\n")

def zero_all(ser: serial.Serial):
    """0x95: 현재 위치를 0점으로 설정"""
    for mid in MOTOR_IDS:
        try:
            send_basic(ser, 0x95, mid)
        except Exception as e:
            print(f"[Warn] Zero(ID={mid}) failed:", e)
    print(">> All motors zeroed.\n")

def disconnect_all(ser: serial.Serial):
    """0x11: 모터 연결 해제"""
    for mid in MOTOR_IDS:
        try:
            send_basic(ser, 0x11, mid)
        except Exception as e:
            print(f"[Warn] Disconnect(ID={mid}) failed:", e)
    print(">> All motors disconnected.\n")

# ====== 5) Present Position 폴링 ======
def read_present_position(ser: serial.Serial, mid: int) -> float:
    """
    0x94: Read single-loop angle
      DATA[0..3] = circleAngle(uint32 LE, 0.01°/LSB)
    """
    for attempt in range(3):
        resp = send_basic(ser, 0x94, mid)
        raw = int.from_bytes(resp[4:8], 'little')
        if 0 <= raw < 36000:
            return raw * 0.01
        print(f"[Warn] ID={mid} invalid raw={raw}, retry {attempt+1}/3")
    raise RuntimeError(f"ID={mid} present position read failed")

def polling_loop(ser: serial.Serial, stop_evt: threading.Event):
    """100 Hz 로 현재 각도를 계속 찍어줍니다."""
    interval = 1.0 / POLL_HZ
    while not stop_evt.is_set():
        t0 = time.time()
        for mid in MOTOR_IDS:
            try:
                pos = read_present_position(ser, mid)
                print(f"[Poll] ID={mid} → {pos:6.2f}°")
            except Exception as e:
                print(f"[Poll] ID={mid} Error:", e)
        dt = interval - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)

# ====== 6) Single-Loop 회전 명령 ======
def rotate_single_loop(
    ser: serial.Serial,
    mid: int,
    angle_deg: float,
    speed: int = DEFAULT_SPEED,
    direction: str = 'CW'
) -> bytearray:
    """
    0xA6: Single-Loop Angle Control 2
      - angle_deg: 도 단위 (0.00~359.99)
      - raw = round(angle_deg*100)  (0.01°/LSB)
      - spin: CW=0x00, CCW=0x01
    """
    spin = 0x00 if direction.upper().startswith('C') else 0x01
    raw  = int(round(angle_deg * 100))
    payload = (
        bytes([spin]) +
        raw.to_bytes(2, 'little') +
        b'\x00' +
        speed.to_bytes(4, 'little')
    )
    return send_split(ser, 0xA6, mid, payload)

# ====== 7) 사용자 인터랙션 ======
def interactive_loop(ser: serial.Serial, stop_evt: threading.Event):
    prompt = (
        "\n명령 형식:\n"
        "  rotate <ID> <angle_deg> [speed] [CW|CCW]\n"
        "    angle_deg: 도 단위 소수점 둘째자리까지\n"
        "    speed: 0.01°/s 단위 (기본 {})\n"
        "  exit 입력 시 종료\n".format(DEFAULT_SPEED)
    )
    print(prompt)
    while not stop_evt.is_set():
        parts = input(">> ").strip().split()
        if not parts: continue
        if parts[0].lower() == 'exit':
            stop_evt.set()
            break
        if parts[0].lower() == 'rotate' and len(parts) >= 3:
            try:
                mid       = int(parts[1])
                angle_deg = float(parts[2])
                speed     = int(parts[3]) if len(parts) > 3 else DEFAULT_SPEED
                direction = parts[4] if len(parts) > 4 else 'CW'
                if mid not in MOTOR_IDS:
                    print("Unknown motor ID"); continue
                rotate_single_loop(ser, mid, angle_deg, speed, direction)
            except Exception as e:
                print("명령 파싱 오류:", e)
        else:
            print("지원되지 않는 명령입니다.")

# ====== 메인 ======
def main():
    parser = argparse.ArgumentParser(description="MG 모터 Single-Loop 제어 + 100Hz 폴링")
    parser.add_argument('-p','--port', default='COM9', help='Serial port')
    parser.add_argument('-b','--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    stop_evt = threading.Event()
    ser = init_serial(args.port, args.baud)

    try:
        connect_all(ser)
        zero_all(ser)

        poll_thr = threading.Thread(target=polling_loop, args=(ser, stop_evt), daemon=True)
        poll_thr.start()

        interactive_loop(ser, stop_evt)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        stop_evt.set()
        disconnect_all(ser)
        ser.close()
        print("프로그램 종료.")

if __name__ == '__main__':
    main()
