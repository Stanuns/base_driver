import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
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
        self.publisher_ = self.create_publisher(UInt8MultiArray, '/dock_ir', 10)

        # 多串口
        self.ports = ["/dev/ttyUSB0","/dev/ttyUSB1", "/dev/ttyUSB2"] #, "/dev/ttyUSB1", "/dev/ttyUSB2"
        self.serial_instances = []

        # 初始化串口
        for port in self.ports:
            try:
                ser = serial.Serial(
                    port=port,
                    baudrate=9600,
                    timeout=1
                )
                self.serial_instances.append({
                    'ser': ser,
                    'buffer': bytearray(),
                    'current_cmd': 0 
                })
                self.get_logger().info("Serial port %s opened successfully." % port)
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {port}, fail: {e}")
                # raise
        if not self.serial_instances:
            self.get_logger().error("No available serial ports, exit!")
            raise RuntimeError("No available serial ports")
        
        self.get_logger().info("Dock Infrared Receiver has started.....")

    def parse_packet(self, buffer):
        """
        解析数据包
        返回: 解析成功返回命令值, 失败返回None
        """
        while True:
            # 查找帧头0xFFFE
            start = buffer.find(b'\xff\xfe')
            if start == -1:
                return None
            # 移除帧头前的垃圾数据
            buffer[:] = buffer[start:]
            # 检查最小包长度
            if len(buffer) < 8:  # 完整包需要8字节
                return None
            # 解析数据长度
            data_length = buffer[2]
            # 检查完整包长度
            total_packet_length = 2 + 1 + data_length + 1  # 头2 + 长度1 + 数据data_length + 尾1
            # 检查数据部分是否完整
            if len(buffer) < total_packet_length:  # 头+长度+数据+尾
                return None
            # 检查帧尾
            if buffer[total_packet_length - 1] != 0x00:
                buffer[:] = buffer[2:]  # 跳过错误帧头
                continue
            # 提取完整数据包
            packet = bytes(buffer[:total_packet_length])
            buffer[:] = buffer[total_packet_length:]  # 移除已处理数据
            # 提取数据部分（功能码 + 命令 + CRC）
            data_part = packet[3:3 + data_length]
            # 计算CRC
            crc_data = data_part[:2]  # 功能码 + 命令
            calculated_crc = calculate_crc(crc_data)
            # 提取接收的CRC
            received_crc = (data_part[3] << 8) | data_part[2]  # 小端模式
            # CRC验证
            if calculated_crc != received_crc:
                self.get_logger().warn("CRC check fail")
                continue
            # 提取命令值
            cmd = data_part[1]
            return cmd

    def process_serial(self):
        """
        循环处理所有串口数据
        """
        while rclpy.ok():
            combined_data = []
            has_new_data = False

            for instance in self.serial_instances:
                ser = instance['ser']
                buffer = instance['buffer']
                current_cmd = instance['current_cmd']

                try:
                    # 读取串口数据
                    data = ser.read(ser.in_waiting or 1)
                    if data:
                        buffer += data
                        self.get_logger().debug(f"From serial port {ser.port} Raw data: {list(data)}")  # 打印原始字节流
                except Exception as e:
                    self.get_logger().error(f"Serial port {ser.port} read fail: {e}")
                    continue

                # 持续解析数据包
                latest_cmd = current_cmd
                while True:
                    cmd = self.parse_packet(buffer)
                    if cmd is None:
                        break
                    latest_cmd = cmd
                    has_new_data = True
                    
                instance['current_cmd'] = latest_cmd
                combined_data.append(latest_cmd)

            # 如果收到新数据则发布组合消息
            if has_new_data:
                msg = UInt8MultiArray()
                msg.data = combined_data
                self.publisher_.publish(msg)
                # self.get_logger().info(
                #     f"发布组合红外数据: [USB0: 0x{combined_data[0]:02X}, "
                #     f"USB1: 0x{combined_data[1]:02X}, "
                #     f"USB2: 0x{combined_data[2]:02X}]"
                # )

def main(args=None):
    rclpy.init(args=args)
    node = IRReceiverNode()
    try:
        node.process_serial()
    except KeyboardInterrupt:
        pass
    finally:
        for instance in node.serial_instances:
            try:
                instance['ser'].close()
            except Exception as e:
                node.get_logger().error(f"Error when close serial port: {e}")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()