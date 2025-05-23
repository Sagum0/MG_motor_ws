import serial
import time

FRAME_HEAD = 0x3E
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200
DEFAULT_POLL_HZ = 60

class MotorSDK:
    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 motor_ids: tuple = (1, 2, 3, 4), poll_hz: int = DEFAULT_POLL_HZ):
        """
        MotorSDK 초기화:
        - port: 직렬 포트 경로
        - baud: 통신 속도
        - motor_ids: 제어할 모터 ID 튜플
        - poll_hz: 각도 폴링 주파수 (Hz)
        """
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.motor_ids = motor_ids
        self.poll_hz = poll_hz
        self._connect_all()

    def _calc_checksum(self, buf: bytearray) -> int:
        return sum(buf) & 0xFF

    def _read_frame(self) -> bytearray:
        ser = self.ser
        while True:
            b = ser.read(1)
            if not b:
                raise RuntimeError('Header(0x3E) Time Out')
            hdr = ser.read(3)
            if len(hdr) < 3:
                raise RuntimeError('Not Enough Header Bytes')
            ln = hdr[2]
            payload = ser.read(ln + 1)
            if len(payload) < ln + 1:
                raise RuntimeError('Not Enough Payload Bytes')
            return bytearray([FRAME_HEAD]) + hdr + payload

    def _send_cmd(self, cmd: int, motor_id: int, data: bytes) -> bytearray:
        header = bytearray([FRAME_HEAD, cmd, motor_id, len(data)])
        header_cs = self._calc_checksum(header)
        data_cs = self._calc_checksum(data)
        pkt = header + bytes([header_cs]) + data + bytes([data_cs])
        self.ser.reset_input_buffer()
        self.ser.write(pkt)
        time.sleep(0.005)
        return self._read_frame()

    def _connect(self, motor_id: int):
        for cmd in (0x1F, 0x12, 0x16, 0x14, 0x10):
            frame = bytearray([FRAME_HEAD, cmd, motor_id, 0])
            frame.append(self._calc_checksum(frame))
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            time.sleep(0.005)
            self._read_frame()

    def _connect_all(self):
        for m in self.motor_ids:
            self._connect(m)
        # 연결 완료

    def read_angle(self, motor_id: int) -> float:
        cmd = 0x94
        frame = bytearray([FRAME_HEAD, cmd, motor_id, 0])
        frame.append(self._calc_checksum(frame))
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        resp = self._read_frame()
        if resp[1] != cmd or resp[2] != motor_id or resp[3] != 4:
            raise RuntimeError('Invalid Read Angle Response')
        raw = int.from_bytes(resp[5:9], 'little')
        return raw / 1000.0

    def read_all_angles(self) -> list:
        return [self.read_angle(m) for m in self.motor_ids]

    def rotate(self, motor_id: int, angle_deg: float, speed_dps: float = 2000.0,
               rev: bool = False) -> bytearray:
        raw_angle = int(angle_deg * 1000)
        raw_speed = int(speed_dps * 100)
        spin = 1 if rev else 0
        data = (bytes([spin]) +
                (raw_angle & 0xFFFFFF).to_bytes(3, 'little') +
                (raw_speed & 0xFFFFFFFF).to_bytes(4, 'little'))
        return self._send_cmd(0xA6, motor_id, data)

    def rotate_all(self, angles: list, speed_dps: float = 2000.0,
                   rev: bool = False):
        for m, a in zip(self.motor_ids, angles):
            self.rotate(m, a, speed_dps, rev)

    def close(self):
        if self.ser.is_open:
            self.ser.close()
