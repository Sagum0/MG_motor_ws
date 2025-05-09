#!/usr/bin/env python3
import argparse, serial, sys, time

FRAME_HEAD = 0x3E

def calc_checksum(buf: bytearray) -> int:
    return sum(buf) & 0xFF

def read_frame(ser: serial.Serial) -> bytearray:
    # 0x3E 헤더 동기화 → hdr(3) → payload(len+1)
    while True:
        b = ser.read(1)
        if not b: raise RuntimeError("헤더(0x3E) 대기 타임아웃")
        if b[0] == FRAME_HEAD: break
    hdr = ser.read(3)
    if len(hdr) < 3: raise RuntimeError("헤더 뒤 바이트 부족")
    ln = hdr[2]
    payload = ser.read(ln+1)
    if len(payload) < ln+1: raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv_split(ser, cmd, motor_id, data):
    """A4/A6 공통: 헤더 CS + 데이터 CS 전송/수신"""
    header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    header_cs = calc_checksum(header)
    data_cs   = calc_checksum(data)
    pkt = header + bytes([header_cs]) + data + bytes([data_cs])

    ser.reset_input_buffer()
    ser.write(pkt)
    print(f"TX(ID={motor_id}):", " ".join(f"{b:02X}" for b in pkt))
    time.sleep(0.005)
    resp = read_frame(ser)
    print(f"RX(ID={motor_id}):", " ".join(f"{b:02X}" for b in resp))
    return resp

def rotate_multi_loop(ser, motor_id, angle_deg, speed_dps=2000.0):
    """Multi-Loop Angle Control 2 (0xA4)"""
    angle_ls = int(round(angle_deg * 1000))  # 0.001° 단위
    speed_ls = int(round(speed_dps * 100))  # 0.01°/s 단위
    data = angle_ls.to_bytes(8,'little',signed=True) + speed_ls.to_bytes(4,'little')
    return send_and_recv_split(ser, 0xA4, motor_id, data)

def rotate_single_loop(ser, motor_id, angle_ctrl, speed_ls=200_000, direction='CW'):
    """Single-Loop Angle Control 2 (0xA6)"""
    spin = 0x00 if direction.upper().startswith('C') else 0x01
    data = bytearray([spin]) \
         + angle_ctrl.to_bytes(2,'little') \
         + b'\x00' \
         + speed_ls.to_bytes(4,'little')
    return send_and_recv_split(ser, 0xA6, motor_id, data)

def connect(ser, motor_id):
    """Ping→productInfo→state3→state2→state1"""
    for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
        # 체크섬 통합 명령 (헤더-only)
        frame = bytearray([FRAME_HEAD, cmd, motor_id, 0])
        frame.append(calc_checksum(frame))
        ser.reset_input_buffer(); ser.write(frame)
        print(f"TX(ID={motor_id}):", " ".join(f"{b:02X}" for b in frame))
        time.sleep(0.005)
        resp = read_frame(ser)
        print(f"RX(ID={motor_id}):", " ".join(f"{b:02X}" for b in resp))
    print(">> Connect 완료!\n")

def main():
    p = argparse.ArgumentParser()
    p.add_argument('-p','--port', default='COM9'); p.add_argument('-b','--baud', type=int, default=115200)
    args = p.parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            # 1) 두 모터(ID=1,2) 모두 Connect
            for mid in (1, 2):
                connect(ser, mid)

            # 2) “동시” 0° 회전 (Multi-Loop)
            for mid in (1, 2):
                rotate_multi_loop(ser, mid, angle_deg=0.0)
            time.sleep(3)

            # 3) “동시” 180° 회전 (Multi-Loop)
            for mid in (1, 2):
                rotate_multi_loop(ser, mid, angle_deg=180.0)

    except serial.SerialException as e:
        print("포트 오류:", e, file=sys.stderr); sys.exit(1)
    except RuntimeError as e:
        print("통신 오류:", e, file=sys.stderr); sys.exit(2)

if __name__ == '__main__':
    main()
