#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MgMotorConnect(Node):
    def __init__(self):
        super().__init__('mg_motor_connect')
        self.get_logger().info(' MG MOTOR CONNECTER IS START ')
        
        self.angle_pub = self.create_publisher(
            Float32MultiArray,
            '/MG_Motor_Angle',
            10
        )
        
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().info(' ')
        
        
def main(args=None):
    rclpy.init(args=args)
    node = MgMotorConnect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()