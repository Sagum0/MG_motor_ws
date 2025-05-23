#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from Motor_SDK import MotorSDK

class MgMotorConnect(Node):
    def __init__(self):
        super().__init__('mg_motor_connect')
        self.get_logger().info('MG MOTOR CONNECTER IS START')

        # 1) 파라미터 선언
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('motor_ids', [1, 2, 3, 4])
        self.declare_parameter('poll_hz', 10)  # 0.1초 간격

        # 2) 파라미터 읽기
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        motor_ids = tuple(self.get_parameter('motor_ids').value)
        poll_hz = self.get_parameter('poll_hz').value

        # 3) SDK 인스턴스 생성 (자동으로 connect_all 호출)
        self.sdk = MotorSDK(port=port, baud=baud, motor_ids=motor_ids, poll_hz=poll_hz)

        # 4) 퍼블리셔 / 타이머 설정
        self.angle_pub = self.create_publisher(Float32MultiArray, '/MG_Motor_Angle', 10)
        timer_period = 1.0 / poll_hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        angles = self.sdk.read_all_angles()
        msg = Float32MultiArray(data=angles)
        self.angle_pub.publish(msg)
        self.get_logger().info(f'Published angles: {angles}')

    def destroy_node(self):
        self.sdk.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MgMotorConnect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
