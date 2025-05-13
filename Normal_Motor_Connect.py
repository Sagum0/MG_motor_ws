# 2초마다 특정 각도만큼 회전하며 Hz 단위로 각도를 monitoring 할 수 있는 코드
#!/usr/bin/env python3

import argparse, serial, sys, time

FRAME_HEAD = 0x3E

# 윈도우용 포트
PORT = 'COM9'
# 우분투용 포트
# PORT = '/dev/ttyUSB0'

MOTOR_ID = (1, 2)
poll_hz = 60

def calc_checksum(buf: bytearray) -> int:
    return sum(buf) & 0xFF

def read_frame(ser: serial.Serial) -> bytearray:
    while True:
        b = ser.read(1)
        if not b:
            raise RuntimeError(' Header(0x3E) Time Out - Runtime Error ')
        
        hdr = ser.read(3)
        
        if len(hdr) < 3:
            raise RuntimeError(' Not Enough Byte before Header ')

        ln = hdr[2]
        payload = ser.read(ln+1)
        
        if len(payload) < ln + 1:
            raise RuntimeError(' Not Enough Data Byte')
        
        return bytearray([FRAME_HEAD]) + hdr + payload
    
def send_and_recv_split(ser, cmd, motor_id, data):
    header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
    header_cs = calc_checksum(header)
    
    data_cs = calc_checksum(data)
    
    pkt = header + bytes([header_cs]) + data + bytes([data_cs])
    
    ser.reset_input_buffer()
    ser.write(pkt)
    
    # print(f" Tx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in pkt))
    time.sleep(0.005)
    
    resp = read_frame(ser)
    # print(f" Rx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in resp))
    
    return resp

def rotate_single_loop(ser, motor_id, angle_deg, speed_dps=2000.0, rev=False):
    """
    UART A6 Single-Loop Angle Control2 변형 포맷:
      DATA[0] = spinDirection (0=NoRev, 1=Rev)
      DATA[1..3] = rawAngle (24bit little-endian, 단위 0.01° → angle_deg*100)
      DATA[4..7] = rawSpeed (32bit little-endian, 단위 0.01dps → speed_dps*100)
    """
    # 1) raw 계산
    raw_angle = int(angle_deg * 1000)   # ex) 1500.00 → 150000
    raw_speed = int(speed_dps * 100)   # ex) 2000.00 → 200000

    # 2) spinDirection
    spin = 1 if rev else 0

    # 3) 바이트 조립 (총 8바이트)
    data = (
        bytes([spin]) +
        (raw_angle  & 0xFFFFFF).to_bytes(3, 'little') +
        (raw_speed & 0xFFFFFFFF).to_bytes(4, 'little')
    )

    # 4) 전송
    return send_and_recv_split(ser, 0xA6, motor_id, data)

def read_single_loop_angle(ser, motor_id):
    """
    Read Single-Loop Angle (CMD=0x94) 명령을 보내고,
    제대로 된 0.01°/LSB 단위 데이터를 읽어 각도로 반환합니다.
    """
    # 1) 요청 프레임 생성
    cmd = 0x94
    frame = bytearray([FRAME_HEAD, cmd, motor_id, 0])
    frame.append(calc_checksum(frame))

    # 2) 전송
    ser.reset_input_buffer()
    ser.write(frame)
    # print(f"Tx(Read SL): {' '.join(f'{b:02X}' for b in frame)}")

    # 3) 응답 수신
    resp = read_frame(ser)
    # print(f"Rx(Read SL): {' '.join(f'{b:02X}' for b in resp)}")

    # 4) 응답 검증
    if resp[1] != cmd or resp[2] != motor_id or resp[3] != 4:
        raise RuntimeError("잘못된 Read SL 응답")

    # 5) 실제 데이터는 resp[5:9] ← resp[4]는 header_cs
    raw = int.from_bytes(resp[5:5+4], 'little', signed=False)
    angle_deg = raw / 1000.0
    
    print(f" ID : {motor_id}         Single-loop angle: {angle_deg:.2f}° ")
    return angle_deg

def connect(ser, motor_id):
    for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
        frame = bytearray([FRAME_HEAD, cmd, motor_id, 0])
        frame.append(calc_checksum(frame))
        
        ser.reset_input_buffer()
        ser.write(frame)
        
        # print(f" Tx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in frame))
        time.sleep(0.005)
        
        resp = read_frame(ser)
        # print(f" Rx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in resp))
    
    print(f" ID : {motor_id} :: Connect Complete! :: ")
    
    
def main():
    p = argparse.ArgumentParser()
    p.add_argument('-p','--port', default=PORT)
    p.add_argument('-b','--baud', type=int, default=115200)
    args = p.parse_args()
    
    motor_angle = []
    interval = 1.0 / poll_hz
    
    count = 0.0
    angle_count = 0.0
    
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            # 연결 단계
            for m_id in MOTOR_ID:
                connect(ser, m_id)
            
            print(" Motor Connect Complete! ")
            
            while True:
                # Polling 을 위한 t0 설정
                t0 = time.perf_counter()
                
                print(" ")
                for m_id in MOTOR_ID:
                    _ = read_single_loop_angle(ser, m_id)
                
                print(count)
                
                    
                dt = time.perf_counter() - t0
                sleep_time = interval - dt
                if sleep_time > 0:
                    time.sleep(sleep_time)
                    
                count += 1
                
                if count % (poll_hz * 2) == 0:
                    if angle_count >= 360:
                        angle_count = 0
                    for m_id in (1, 2):
                        rotate_single_loop(ser, m_id, angle_deg=angle_count, speed_dps=2000.0, rev=False)
                    angle_count += 120
                
                
    except serial.SerialException as e:
        print("포트 오류:", e, file=sys.stderr); sys.exit(1)
    except RuntimeError as e:
        print("통신 오류:", e, file=sys.stderr); sys.exit(2)
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == '__main__':
    main()