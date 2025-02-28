import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import serial
import struct

def calculate_crc(data):
    """
    MODBUS CRC16校验计算
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

class IRReceiverNode(Node):
    def __init__(self):
        super().__init__('ir_receiver_node')
        self.publisher_ = self.create_publisher(UInt8, '/dock_ir_2', 10)
        
        # 初始化串口
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=9600,
            timeout=1
        )
        self.buffer = bytearray()
        self.get_logger().info("Dock Infrared Receiver has started.....")

    def parse_packet(self):
        """
        解析数据包
        返回: 解析成功返回命令值，失败返回None
        """
        while True:
            # 查找帧头0xFFFE
            start = self.buffer.find(b'\xff\xfe')
            if start == -1:
                return None
            
            # 移除帧头前的垃圾数据
            self.buffer = self.buffer[start:]
            
            # 检查最小包长度
            if len(self.buffer) < 8:  # 完整包需要8字节
                return None
            
            # 解析数据长度
            data_length = self.buffer[2]
            
            # 检查数据部分是否完整
            if len(self.buffer) < 2 + 1 + data_length + 1:  # 头+长度+数据+尾
                return None
            
            # 检查帧尾
            if self.buffer[2 + 1 + data_length] != 0x00:
                self.buffer = self.buffer[2:]  # 跳过错误帧头
                continue
                
            # 提取完整数据包
            packet = self.buffer[:2 + 1 + data_length + 1]
            self.buffer = self.buffer[2 + 1 + data_length + 1:]  # 移除已处理数据
            
            # 提取数据部分（功能码 + 命令 + CRC）
            data_part = packet[3:3 + data_length]
            
            # 计算CRC
            crc_data = data_part[:2]  # 功能码 + 命令
            calculated_crc = calculate_crc(crc_data)
            
            # 提取接收的CRC
            received_crc = (data_part[3] << 8) | data_part[2]  # 小端模式
            
            # CRC验证
            if calculated_crc != received_crc:
                self.get_logger().warn("CRC校验失败")
                continue
                
            # 提取命令值
            cmd = data_part[1]
            return cmd
            
        return None

    def process_serial(self):
        """
        处理串口数据
        """
        while rclpy.ok():
            # 读取串口数据
            data = self.ser.read(self.ser.in_waiting or 1)
            if data:
                self.buffer += data
                
                # 持续解析数据包
                while True:
                    cmd = self.parse_packet()
                    if cmd is None:
                        break
                    
                    # 发布消息
                    msg = UInt8()
                    msg.data = cmd
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"发布红外命令: 0x{cmd:02X}")

def main(args=None):
    rclpy.init(args=args)
    node = IRReceiverNode()
    
    try:
        node.process_serial()
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()