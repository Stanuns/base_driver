# 浙江华麦机器人底盘接口
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
import serial
import struct
from robot_interfaces.msg import HMAutoDockState
from robot_interfaces.msg import HMAutoDockTrigger
import time
import traceback
from tf_transformations import quaternion_from_euler

class BaseSerialError(Exception):
    def __init__(self, message=""):
        self.message = message
        super(BaseSerialError,self).__init__()

class HmBaseNode(Node):
    def __init__(self):
        super().__init__('hm_serial_node')
        
        self.control_data_head = b'\xAA\x55'
        self.control_data_foot = b'\x88'

        # 初始化下位机串口连接
        self.ser_base = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        if not self.ser_base.is_open:
            self.get_logger().error("Failed to open serial port!")
            raise Exception("Serial port open failed")

        # 初始化Android串口连接
        self.ser_android = serial.Serial(
            port='/dev/hm_android',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        if not self.ser_android.is_open:
            self.get_logger().error("Failed to open android serial port!")
            raise Exception("Android serial port open failed")

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.handle_hm_cmd_vel,
            10
        )
        self.subscription2 = self.create_subscription(
            HMAutoDockTrigger,
            '/hm_auto_dock_trigger',
            self.handle_hm_auto_dock,
            10
        )

        # publisher for /odom
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # New publisher for /hm_dock_state
        self.dock_state_publisher = self.create_publisher(
            HMAutoDockState,
            '/hm_auto_dock_state',
            10
        )

        # New publisher for /android_voice_action
        self.android_voice_action_publisher = self.create_publisher(
            UInt8,
            '/android_voice_action',
            10
        )

        self.get_logger().info("Node initialized")

    # 处理hm_cmd_vel
    def handle_hm_cmd_vel(self, msg):
        """处理速度命令的回调函数"""
        try:
            # 消息提取
            self.get_logger().info("---handle_hm_cmd_vel---")
            linear_action = msg.linear.x
            angular_action = msg.angular.z
            action_code = 0x00
            if linear_action > 0 and angular_action == 0:
                action_code = 4
            elif linear_action < 0 and angular_action == 0:
                action_code = 3
            elif linear_action == 0 and angular_action > 0:
                action_code = 2
            elif linear_action == 0 and angular_action < 0:
                action_code = 1
            self.send_speed_approximately(action_code)
        except ValueError as e:
            self.get_logger().warn(f"Invalid value: {e}")
    
    # 处理hm_auto_dock
    def handle_hm_auto_dock(self, msg):
        """处理"""
        try:
            # 消息提取
            self.get_logger().info("---handle_hm_auto_dock---")
            action_code = msg.action
            self.send_hm_auto_dock(action_code)
        except ValueError as e:
            self.get_logger().warn(f"Invalid value: {e}")

    ### 控制信息发送至底盘下位机
    def send_speed_approximately(self, action_value):
        """构建并发送数据帧"""
        try:
            # 校验动作值范围
            if not (0 <= action_value <= 255):
                raise ValueError("Action value out of range (0-255)")

            # 构建数据帧
            frame = bytes()
            # 帧长（固定值）
            frame += struct.pack('BB', 0x00, 0x07)
            # 命令码 + 流水号
            frame += struct.pack('BB', 0x22, 0x00)
            # 系列编号 + 动作值
            frame += struct.pack('BB', 0x01, action_value)
            # 运动时间（小端模式 0x0320 = 800ms）
            frame += struct.pack('<H', 800)  # 小端模式打包

            # 发送数据
            self._send_data_frame(frame)
            self.get_logger().info(f"Sent speed_approximately frame to base : 0x{action_value:02X}")

        except Exception as e:
            self.get_logger().error(f"Speed_approximately Frame sending to base failed: {str(e)}")
    
    def send_hm_auto_dock(self, action_value):
        """构建并发送数据帧"""
        try:
            # 校验动作值范围
            if not (0 <= action_value <= 255):
                raise ValueError("Action value out of range (0-255)")

            # 构建数据帧
            frame = bytes()
            # 帧长（固定值）
            frame += struct.pack('BB', 0x00, 0x04)
            # 命令码 + 流水号
            frame += struct.pack('BB', 0x27, 0x00)
            # 命令
            frame += struct.pack('B', action_value)

            # 发送数据
            self._send_data_frame(frame)
            self.get_logger().info(f"Sent hm_auto_dock frame to base: 0x{action_value:02X}")

        except Exception as e:
            self.get_logger().error(f"Hm_auto_dock Frame sending to base failed: {str(e)}")

    def send_speed(self, left_speed, right_speed):
        if left_speed >= 0:
            left_dir = b'\x01'
        else:
            left_dir = b'\x00'
        if right_speed >= 0:
            right_dir = b'\x01'
        else:
            right_dir = b'\x00'
        sleft = abs(left_speed)
        sright = abs(right_speed)

        # 构建数据帧
        frame = bytes()
        # 帧长（固定值）
        frame += struct.pack('BB', 0x00, 0x09)
        # 命令码 + 流水号
        frame += struct.pack('BB', 0x77, 0x00)
        # 左右轮速度及方向
        frame += struct.pack('<H', sleft)
        frame += struct.pack('<H', sright)
        frame += (left_dir + right_dir)

        self._send_data_frame(frame)
        self.get_logger().info(f"Sent wheel speed frame to base")

    def _send_data_frame(self, data_tob_send): 
        control_data = self.control_data_head + data_tob_send + self.control_data_foot 
        try: 
            self.ser_base.write(control_data)     
        except serial.SerialTimeoutException as e:
            self.get_logger().error(f"write time out, so flush the output: {str(e)}")
            self.ser_base.flushOutput()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Port may disconnected, try to reopen the port. error: {str(e)}")
            traceback.print_exc()
            return None 
    
    ### 读取底盘下位机返回信息
    def read_base_serial_data(self):
        """读取串口数据并处理"""
        while rclpy.ok():
            # # self.get_logger().info(f"----read_base_serial_data----")
            # if self.ser_base.in_waiting > 0:
            #     # 读取帧头
            #     header = self.ser_base.read(2)

            #     ## 读取回充状态
            #     if header == b'\xAA\x55':
            #         self.get_logger().info(f"--------------read_base_serial_data--Have read header----------------")
            #         # 读取帧长
            #         frame_length = self.ser_base.read(2)
            #         if frame_length == b'\x00\x04':
            #             # 读取命令码、流水号、系列编号、返回结果
            #             data = self.ser_base.read(4)
            #             if len(data) == 4:
            #                 command_code, sequence_number, series_number, result = struct.unpack('BBBB', data)
            #                 if command_code == 0x27 and sequence_number == 0x00 and series_number == 0x02:
            #                     # 读取帧尾
            #                     footer = self.ser_base.read(1)
            #                     if footer == b'\x88':
            #                         # 发布返回结果到 /hm_dock_state
            #                         msg = UInt8()
            #                         msg.data = result
            #                         self.dock_state_publisher.publish(msg)
            #                         self.get_logger().info(f"Received dock state: {result}")
            #                     else:
            #                         self.get_logger().warn("Invalid frame footer")
            #                 else:
            #                     self.get_logger().warn("Invalid command code or series number")
            #             else:
            #                 self.get_logger().warn("Incomplete data frame")
            #         else:
            #             self.get_logger().warn("Invalid frame length")
            #     else:
            #         self.get_logger().warn("Invalid frame header")
            data = self.get_base_new_data()
            if data is None:
                pass
                return
            income_data_type = data.get('type')
            if income_data_type == 'odom':
                self.cast_odom(data)
            elif quaternion_from_euler =='auto_dock':
                self.cast_dock_state(data)
            else: 
                return

    def get_base_new_data(self):
        try:
            frame = self.read_base_raw_frame()
            code = frame.get('code')
            data_in_frame = frame.get('data')
            if code == 78: #odom
                return self._case_odom(data_in_frame)
            elif code == 27: #回充反馈
                return self._case_auto_dock(data_in_frame)
        except BaseSerialError as e:
            self.get_logger().error(f"Read serial time out. {str(e)}")
            return None

    def read_base_raw_frame(self):
        frame = {}
        e = BaseSerialError()
        if self.ser_base.in_waiting > 0:
            header = self.ser_base.read_until(terminator=b'\xAA\x55')
        
        if (len(header)>2) and (header != b'\xAA\x55'):
            e.message = 'Frame corrupted, head can not be reached in time.'
            raise e
        
        zero = self.ser_base.read(1)
        if zero != b'\x00':
            e.message = 'Error occur after head reached.'
            raise e

        len_raw = self.ser_base.read(1)
        if len(len_raw) != 1:
            e.message = 'Size byte does not arrive in time.'
            raise e
        data_len = ord(len_raw)

        code_raw = self.ser_base.read(1)
        if len(code_raw) != 1:
            e.message = 'Command code does not arrive in time.'
            raise e
        code = ord(code_raw)  # command code

        sequence_raw = self.ser_base.read(1)
        if sequence_raw != b'\x00':
            e.message = 'Sequence number code does not arrive in time.'
            raise e

        #fix bugs from 华麦
        if code == 27:
            data_raw =  self.ser_base.read(data_len-2) #帧长值华麦从底盘上传错误
        else:
            data_raw =  self.ser_base.read(data_len-3) #帧长-1命令码-1流水号-1帧尾

        footer = self.ser_base.read(1)
        if footer != b'\x88':
            e.message = 'No footer'
            raise e

        frame['data'] = data_raw  # bytes
        frame['code'] = code  # int
        return frame

    def _case_odom(data_raw): 
        if len(data_raw) != 20:  # refer to the data sheet provided by jinfei
            e = BaseSerialError()
            e.message = "Odom data corrupted"
            raise e
        data = {'type': 'odom'}
        x, y, yaw, v, w = struct.unpack('<5f', data_raw)
        data['x'] = x
        data['y'] = y
        data['yaw'] = yaw
        data['v'] = v
        data['w'] = w 
        return data
    def _case_auto_dock(data_raw):
        if len(data_raw) != 2:  # refer to the data sheet provided by jinfei
            e = BaseSerialError()
            e.message = "dock state data corrupted"
            raise e
        data = {'type': 'auto_dock'}
        series_number, state = struct.unpack('BB', data_raw)
        data['series_number'] = series_number
        data['state'] = state
        return data


    def cast_odom(self, data):
        x = data.get('x')
        y = data.get('y')
        yaw = data.get('yaw')
        v = data.get('v')
        w = data.get('w')
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        self.odom_publisher.publish(msg)
    
    def cast_dock_state(self, data):
        msg = HMAutoDockState()
        msg.header.frame_id = 'auto_dock_state'
        msg.header.stamp = self.get_clock().now().to_msg()
        state = data.get('state')
        msg.state = state
        self.dock_state_publisher.publish(msg)
        

    ### 读取Android pad返回信息
    def read_android_serial_data(self):
        """读取android串口数据并处理"""
        while rclpy.ok():
            if self.ser_android.in_waiting > 0:
                # 读取帧头
                header = self.ser_android.read(2)
                if header == b'\xAA\x55':
                    self.get_logger().info(f"--------------read_android_serial_data--Have read header----------------")
                    # 读取帧长
                    frame_length = self.ser_android.read(2)

                    # 模糊运动控制
                    if frame_length == b'\x00\x07':
                        # 读取命令码、流水号、系列编号、动作值、运动时间
                        data = self.ser_android.read(6)
                        if len(data) == 6:
                            command_code, sequence_number, series_number, action_value, motion_time = struct.unpack('BBBBH', data)
                            if command_code == 0x22 and sequence_number == 0x00 and series_number == 0x01:
                                # 读取帧尾
                                footer = self.ser_android.read(1)
                                if footer == b'\x88':
                                    # 发布动作值到 /android_voice_action
                                    msg = UInt8()
                                    msg.data = action_value
                                    self.android_voice_action_publisher.publish(msg)
                                    # 写入android语音识别的数据到写入下位机串口
                                    # 重复写防止下位机没有反应
                                    for i in range(1,3):
                                        self.send_speed_approximately(action_value)
                                        time.sleep(0.02)
                                    self.get_logger().info(f"Received android voice action | send_speed_approximately: {action_value}")
                                else:
                                    self.get_logger().warn("Invalid frame footer | send_speed_approximately")
                            else:
                                self.get_logger().warn("Invalid command code or series number | send_speed_approximately")
                        else:
                            self.get_logger().warn("Incomplete data frame | send_speed_approximately")

                    #回充启停控制
                    elif frame_length == b'\x00\x04':
                        data = self.ser_android.read(3)
                        if len(data) == 3:
                            command_code, sequence_number, command = struct.unpack('BBB', data)
                            if command_code == 0x27 and sequence_number == 0x00 :
                                # 读取帧尾
                                footer = self.ser_android.read(1)
                                if footer == b'\x88':
                                    # 发布动作值到 /android_voice_action
                                    msg = UInt8()
                                    msg.data = command
                                    self.android_voice_action_publisher.publish(msg)
                                    # 写入android语音识别的数据到写入下位机串口
                                    # 重复写防止下位机没有反应
                                    for i in range(1,3):
                                        self.send_hm_auto_dock(command)
                                        time.sleep(0.02)
                                    self.get_logger().info(f"Received android voice action | send_hm_auto_dock: {command}")
                                else:
                                    self.get_logger().warn("Invalid frame footer | send_hm_auto_dock")
                            else:
                                self.get_logger().warn("Invalid command code or series number | send_hm_auto_dock")
                        else:
                            self.get_logger().warn("Incomplete data frame | send_hm_auto_dock")

                    else:
                        self.get_logger().warn("Read Android Invalid frame length")
                else:
                    self.get_logger().warn("Read Android Invalid frame header")

    def __del__(self):
        """析构时关闭串口"""
        if hasattr(self, 'ser_base') and self.ser_base.is_open:
            self.ser_base.close()
        # if hasattr(self, 'ser_android') and self.ser_android.is_open:
        #     self.ser_android.close()

def main(args=None):
    rclpy.init(args=args)
    node = HmBaseNode()
    try:
        # Start reading serial data in a separate thread
        import threading
        serial_thread = threading.Thread(target=node.read_base_serial_data)
        serial_thread.daemon = True
        serial_thread.start()

        # Start reading android serial data in a separate thread
        android_serial_thread = threading.Thread(target=node.read_android_serial_data)
        android_serial_thread.daemon = True
        android_serial_thread.start()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()