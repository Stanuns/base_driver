# 浙江华麦机器人底盘接口
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class HmBaseNode(Node):
    def __init__(self):
        super().__init__('hm_serial_node')
        
        # 初始化串口连接
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        if not self.ser.is_open:
            self.get_logger().error("Failed to open serial port!")
            raise Exception("Serial port open failed")

        # 创建订阅者
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.handle_cmd_vel,
            10
        )
        self.get_logger().info("Node initialized")

    def handle_cmd_vel(self, msg):
        """处理速度命令的回调函数"""
        try:
            # 从消息中提取动作值（示例使用linear.x的整数部分）
            self.get_logger().info("---handle_cmd_vel---")
            action = msg.linear.x
            action_code = 0x00
            if action > 0:
                action_code = 4
            elif action < 0:
                action_code = 3
            # action_code = 0x04
            self.send_serial_frame(action_code)
        except ValueError as e:
            self.get_logger().warn(f"Invalid action value: {e}")

    def send_serial_frame(self, action_value):
        # self.get_logger().info("---send_serial_frame, action_value: %d" % action_value)
        """构建并发送数据帧"""
        try:
            # 校验动作值范围
            if not (0 <= action_value <= 255):
                raise ValueError("Action value out of range (0-255)")

            # 构建数据帧
            frame = bytes()
            # 帧头
            frame += struct.pack('BB', 0xAA, 0x55)
            # 帧长（固定值）
            frame += struct.pack('BB', 0x00, 0x07)
            # 命令码 + 流水号
            frame += struct.pack('BB', 0x22, 0x00)
            # 系列编号 + 动作值
            frame += struct.pack('BB', 0x01, action_value)
            # 运动时间（小端模式 0x0320 = 800ms）
            frame += struct.pack('<H', 800)  # 小端模式打包
            # 帧尾
            frame += struct.pack('B', 0x88)

            # 发送数据
            self.ser.write(frame)
            self.get_logger().info(f"Sent frame with action: 0x{action_value:02X}")

        except Exception as e:
            self.get_logger().error(f"Frame sending failed: {str(e)}")

    def __del__(self):
        """析构时关闭串口"""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = HmBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
