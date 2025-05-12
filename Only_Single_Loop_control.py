#!/usr/bin/env python3
import serial, sys, time

FRAME_HEAD = 0x3E

def calc_checksum(buf: bytearray) -> int:
    return sum(buf) & 0xFF

def read_frame(ser: serial.Serial) -> bytearray:
    while True:
        b = ser.read(1)
        if not b: raise RuntimeError("헤더(0x3E) 대기 타임아웃")
        if b[0] == FRAME_HEAD: break
    hdr = ser.read(3)
    if len(hdr) < 3: raise RuntimeError("헤더 뒤 바이트 부족")
    ln = hdr[2]
    payload = ser.read(ln + 1)
    if len(payload) < ln + 1: raise RuntimeError("데이터 바이트 부족")
    return bytearray([FRAME_HEAD]) + hdr + payload

def send_and_recv(ser, cmd, motor_id, data):
    header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    packet = header + bytes([calc_checksum(header)]) + data + bytes([calc_checksum(data)])
    ser.reset_input_buffer(); ser.write(packet)
    print("TX:", packet.hex(' ').upper())
    time.sleep(0.005)
    resp = read_frame(ser)
    print("RX:", resp.hex(' ').upper())
    return resp

def connect(ser, motor_id):
    for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
        frame = bytearray([FRAME_HEAD, cmd, motor_id, 0x00])
        frame.append(calc_checksum(frame))
        ser.reset_input_buffer(); ser.write(frame)
        print("TX:", frame.hex(' ').upper())
        time.sleep(0.005)
        resp = read_frame(ser)
        print("RX:", resp.hex(' ').upper())
    print(f">> Motor {motor_id} 연결 완료\n")

def rotate_and_read(ser, motor_id, angle_deg, speed_dps, direction):
    spin = 0x00 if direction.upper().startswith("C") else 0x01
    angle_ls = int(round(angle_deg * 100))
    speed_ls = int(round(speed_dps * 100))
    data = bytearray([spin]) + angle_ls.to_bytes(3,'little',signed=True) + speed_ls.to_bytes(4,'little')
    send_and_recv(ser, 0xA6, motor_id, data)
    time.sleep(0.1)
    # Read angle
    frame = bytearray([FRAME_HEAD, 0x94, motor_id, 0x00])
    frame.append(calc_checksum(frame))
    ser.reset_input_buffer(); ser.write(frame)
    print("TX:", frame.hex(' ').upper())
    time.sleep(0.005)
    resp = read_frame(ser)
    raw = int.from_bytes(resp[5:9], 'little', signed=True)
    print(f"측정 각도: {raw/100:.2f}°")

def main():
    port = input("시리얼 포트 (기본 COM9): ") or "COM9"
    baud = input("보드레이트 (기본 115200): ")
    baud = int(baud) if baud else 115200

    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            mid = int(input("모터 ID 입력: "))
            spd = float(input("속도(deg/s) 입력: "))
            ang = float(input("목표각(deg) 입력: "))
            dir = input("회전 방향(CW/CCW): ").strip().upper()
            print()

            connect(ser, mid)
            rotate_and_read(ser, mid, ang, spd, dir)

    except serial.SerialException as e:
        print("포트 오류:", e, file=sys.stderr); sys.exit(1)
    except RuntimeError as e:
        print("통신 오류:", e, file=sys.stderr); sys.exit(2)

if __name__ == '__main__':
    main()
