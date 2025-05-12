#!/usr/bin/env python3

import argparse, serial, sys, time

FRAME_HEAD = 0x3E

# 윈도우용 포트
PORT = 'COM9'
# 우분투용 포트
# PORT = '/dev/ttyUSB0'

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
    header = bytearray([FRAME_HEAD], cmd, motor_id, len(data))
    header_cs = calc_checksum(header)
    
    data_cs = calc_checksum(data)
    
    pkt = header + bytes([header_cs]) + data + bytes([data_cs])
    
    ser.reset_input_buffer()
    ser.write(pkt)
    
    print(f" Tx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in pkt))
    time.sleep(0.005)
    
    resp = read_frame(ser)
    print(f" Rx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in resp))
    
    return resp

def rotate_single_loop(ser, motor_id, angle_ctrl, speed_ls=200_000, direction = 'CW'):
    # 200_000 -> 200,000 과 같은 내용임 파이썬에서 허용된 높은 자리 숫자 호출하는 방식임. 즉 20만
    spin = 0x00 if direction.upper().startswith('C') else 0x01
    
    data = bytearray([spin]) \
            + angle_ctrl.to_bytes(2, 'little') \
            + b'\x00' \
            + speed_ls.to_bytes(4,'little')
            
    return send_and_recv_split(ser, 0xA6, motor_id, data)

def connect(ser, motor_id):
    for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
        frame = bytearray([FRAME_HEAD, cmd, motor_id, 0])
        frame.append(calc_checksum(frame))
        
        ser.reset_input_buffer()
        ser.write(frame)
        
        print(f" Tx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in frame))
        time.sleep(0.005)
        
        resp = read_frame(ser)
        print(f" Rx(ID = {motor_id}) : "," ".join(f"{b:02X}" for b in resp))
    
    print(" << Connect Complete! >>")
    
def main():
    p = argparse.ArgumentParser()
    p.add_argument('-p', '--port', default=PORT)
    p.add_argument('-b','--baud', type=int, default=115200)
    args = p.parse_args()
    
    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
            for m_id in (1, 2):
                connect(ser, m_id)
                
            for m_id in (1, 2):
                rotate_single_loop(ser, m_id, angle_deg = 0.0)
                
            time.sleep(3)
            
            for m_id in (1, 2):
                rotate_single_loop(ser, m_id, angle_deg = 180.0)
                
    except serial.SerialException as e:
        print("포트 오류:", e, file=sys.stderr); sys.exit(1)
    except RuntimeError as e:
        print("통신 오류:", e, file=sys.stderr); sys.exit(2)

if __name__ == '__main__':
    main()
