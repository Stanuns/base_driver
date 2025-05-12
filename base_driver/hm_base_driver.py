# 浙江华麦机器人底盘接口
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import struct
from robot_interfaces.msg import HMAutoDockState
from robot_interfaces.msg import HMAutoDockTrigger
import time
import traceback
from tf_transformations import quaternion_from_euler
import math
# import sys
# import os
# sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

# from covariances import ODOM_POSE_COVARIANCE

class BaseSerialError(Exception):
    def __init__(self, message=""):
        self.message = message
        super(BaseSerialError,self).__init__()

class HmBaseNode(Node):
    def __init__(self):
        super().__init__('hm_base_node')
        #增加 parameters server
        self.declare_parameter('max_linear_speed', 0.01)  # Default value as fallback
        self.max_linear_speed_ = self.get_parameter('max_linear_speed').value
        self.get_logger().info(f"From base.yaml loaded max_linear_speed: {self.max_linear_speed_}")
        # new_param = rclpy.parameter.Parameter(
        #     'max_linear_speed',
        #     rclpy.Parameter.Type.DOUBLE,
        #     0.01
        # )
        # all_new_parameters = [new_param]
        # self.set_parameters(all_new_parameters)
                # 强制同步参数到服务器（确保参数可见）
        # self.set_parameters([rclpy.Parameter(
        #     'max_linear_speed',
        #     rclpy.Parameter.Type.DOUBLE,
        #     self.max_linear_speed_
        # )])
        # self.get_logger().info(f"Parameter set: max_linear_speed={self.max_linear_speed_}")

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.wheelSeparate = 0.134
        self.wheelRadius = 0.03

        self.ODOM_POSE_COVARIANCE = [float(x) for x in [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.03]]
        self.ODOM_POSE_COVARIANCE2 = [float(x) for x in [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.03]]

        self.ODOM_TWIST_COVARIANCE = [float(x) for x in [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.03]]
        self.ODOM_TWIST_COVARIANCE2 = [float(x) for x in [0.001, 0.0, 0.0, 0.0, 0.0, 0.0, 
                        0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.03]]

        self.cmd_vel_code = 0x00
        self.run_time = 0

        # self.cmd_vel_exec_tag = True
        self.count_0x00 = 0
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.control_data_head = b'\xAA\x55'
        self.control_data_foot = b'\x88'

        self.is_near_dock = 0

        # 初始化下位机串口连接
        self.ser_base = serial.Serial(
            port='/dev/ttyUSB1',
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
            port='/dev/ttyUSB2',
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
            self.handle_cmd_vel, #self.handle_hm_cmd_vel handle_cmd_vel
            10
        )
        # self.subscription = self.create_subscription(
        #     Twist,
        #     '/cmd_vel_nav',
        #     self.handle_hm_cmd_vel_nav,
        #     10
        # )
        self.subscription2 = self.create_subscription(
            HMAutoDockTrigger,
            '/hm_auto_dock_trigger',
            self.handle_hm_auto_dock,
            10
        )

        self.subscription3 = self.create_subscription(
            UInt8,
            '/is_near_dock',
            self.handle_is_near_dock,
            10
        )

        # publisher for /odom
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.imu_publisher = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )

        # New publisher for /hm_dock_state
        self.dock_state_publisher = self.create_publisher(
            HMAutoDockState,
            '/hm_auto_dock_state',
            10
        )

        # New publisher for /android_voice_action
        self.android_voice_motion_publisher = self.create_publisher(
            UInt8,
            '/android_voice_motion',
            10
        )
        self.android_voice_dock_publisher = self.create_publisher(
            UInt8,
            '/android_voice_dock',
            10
        )
        self.nav_to_goal_recycle_publisher = self.create_publisher(
            UInt8,
            '/nav_to_goal_recycle_trigger',
            10
        )

        self.get_logger().info("Node initialized")

    
    # # 处理cmd_vel_nav
    # def handle_hm_cmd_vel_nav(self, msg):
    #     try:
    #         self.cmd_vel_exec_tag = False
    #         # 消息提取
    #         self.get_logger().info("---handle_hm_cmd_vel_nav---")
    #         linear_velocity = msg.linear.x
    #         angular_velocity = msg.angular.z
    #         cmd_vel_nav_code = 0x00
    #         # run_time = 200
    #         if linear_velocity > 0.025*2 and abs(angular_velocity) < 0.05*2: #前进
    #             cmd_vel_nav_code = 3
    #             self.send_speed_approximately(cmd_vel_nav_code, 80)
    #             time.sleep(0.08)
    #         elif linear_velocity < -0.025*2 and abs(angular_velocity) < 0.05*2: #后退
    #             cmd_vel_nav_code = 4
    #             self.send_speed_approximately(cmd_vel_nav_code, 80)
    #             time.sleep(0.08)
    #         elif abs(linear_velocity) < 0.025*2 and angular_velocity > 0.05*2: #逆时针
    #             cmd_vel_nav_code = 2
    #             self.send_speed_approximately(cmd_vel_nav_code, 20)
    #             time.sleep(0.02)
    #         elif abs(linear_velocity) < 0.025*2 and angular_velocity < -0.05*2: #顺时针
    #             cmd_vel_nav_code = 1
    #             self.send_speed_approximately(cmd_vel_nav_code, 20)
    #             time.sleep(0.02)
    #         elif linear_velocity > 0.025*2 and angular_velocity > 0.05*2: #前进 逆时针
    #             # cmd_vel_nav_code = 3
    #             # self.send_speed_approximately(cmd_vel_nav_code, 40)
    #             # time.sleep(0.04)
    #             cmd_vel_nav_code = 2
    #             self.send_speed_approximately(cmd_vel_nav_code, 10)
    #             time.sleep(0.01)
    #         elif linear_velocity > 0.025*2 and angular_velocity < -0.05*2: #前进 顺时针
    #             # cmd_vel_nav_code = 3
    #             # self.send_speed_approximately(cmd_vel_nav_code, 40)
    #             # time.sleep(0.04)
    #             cmd_vel_nav_code = 1
    #             self.send_speed_approximately(cmd_vel_nav_code, 10)
    #             time.sleep(0.01)
    #         elif linear_velocity < -0.025*2 and angular_velocity > 0.05*2: #后退 逆时针
    #             # cmd_vel_nav_code = 4
    #             # self.send_speed_approximately(cmd_vel_nav_code, 40)
    #             # time.sleep(0.04)
    #             cmd_vel_nav_code = 2
    #             self.send_speed_approximately(cmd_vel_nav_code, 10)
    #             time.sleep(0.01)
    #         elif linear_velocity < -0.025*2 and angular_velocity < -0.05*3.4: #后退 顺时针
    #             # cmd_vel_nav_code = 4
    #             # self.send_speed_approximately(cmd_vel_nav_code, 40)
    #             # time.sleep(0.04)
    #             cmd_vel_nav_code = 1
    #             self.send_speed_approximately(cmd_vel_nav_code, 10)
    #             time.sleep(0.01)
    #         elif abs(linear_velocity) <= 0.025*2 and abs(angular_velocity) <= 0.05*3.4:
    #             cmd_vel_nav_code = 0
    #             self.send_speed_approximately(cmd_vel_nav_code, 80)
    #             time.sleep(0.08)
            
    #     except ValueError as e:
    #         self.get_logger().warn(f"Invalid value: {e}")

    # 处理hm_cmd_vel，只能给出基本前后左右及运动时间，无法指定速度
    def handle_hm_cmd_vel(self, msg):
        """处理速度命令的回调函数"""
        try:
            # 消息提取
            # self.cmd_vel_exec_tag = True
            # self.get_logger().info("---handle_hm_cmd_vel---")
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z
            self.cmd_vel_code = 0x00
            self.run_time = 20
            if linear_velocity > 0.025 and abs(angular_velocity) < 0.05*2: #前进
                self.cmd_vel_code = 3
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif linear_velocity < -0.025 and abs(angular_velocity) < 0.05*2: #后退
                self.cmd_vel_code = 4
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif abs(linear_velocity) < 0.025 and angular_velocity > 0.05*2: #逆时针
                self.cmd_vel_code = 2
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif abs(linear_velocity) < 0.025 and angular_velocity < -0.05*2: #顺时针
                self.cmd_vel_code = 1
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif linear_velocity > 0.025 and angular_velocity > 0.05*2: #前进 逆时针
                self.cmd_vel_code = 2
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif linear_velocity > 0.025 and angular_velocity < -0.05*2: #前进 顺时针
                self.cmd_vel_code = 1
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif linear_velocity < -0.025 and angular_velocity > 0.05*2: #后退 逆时针
                self.cmd_vel_code = 2
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif linear_velocity < -0.025 and angular_velocity < -0.05*2: #后退 顺时针
                self.cmd_vel_code = 1
                self.run_time = 80
                # self.send_speed_approximately(self.cmd_vel_code, self.run_time)
            elif abs(linear_velocity) <= 0.025 and abs(angular_velocity) <= 0.05*2:
                self.cmd_vel_code = 0
                self.run_time = 80
                # for i in range(1,3):
                #     self.send_speed_approximately(self.cmd_vel_code, self.run_time)
                #     time.sleep(self.run_time/1000)
            self.send_speed_approximately(self.cmd_vel_code, self.run_time)
        except ValueError as e:
            self.get_logger().warn(f"Invalid value: {e}")

    # def cmd_vel_continue(self):
    #     while rclpy.ok():
    #         # self.get_logger().info(f"self.cmd_vel_code : 0x{self.cmd_vel_code:02X}, {self.cmd_vel_code != 0x00}")
    #         if self.cmd_vel_exec_tag:
    #             if self.cmd_vel_code != 0x00:
    #                 self.count_0x00 = 0
    #                 self.send_speed_approximately(self.cmd_vel_code, self.run_time)
    #             elif self.cmd_vel_code == 0x00 and self.count_0x00 < 3:
    #                 self.send_speed_approximately(self.cmd_vel_code, self.run_time)
    #                 self.count_0x00 += 1
    #         time.sleep(self.run_time/1000)
    #     # pass

    #速度下发指令
    def handle_cmd_vel(self, msg):
        """处理速度命令的回调函数"""
        try:
            # self.get_logger().info("---handle_cmd_vel---")
            linear_velocity = msg.linear.x
            angular_velocity = msg.angular.z

            #由于华麦底盘与上层控制指令相反,需处理
            linear_velocity = -linear_velocity
            angular_velocity = angular_velocity*1.5

            #线速度角速度转换成左右轮线速度由华麦提供
            # self.wheelSeparate = 0.134
            # self.wheelRadius = 0.03
            left_speed = linear_velocity - 0.5 * self.wheelSeparate * angular_velocity
            right_speed = linear_velocity + 0.5 * self.wheelSeparate * angular_velocity
            #由于华麦底盘轮子的最大线速度只有0.23m/s,需做限制,等比例限制
            MAX_SPEED = 0.23  # 最大线速度绝对值限制(m/s)
            abs_left = abs(left_speed)
            abs_right = abs(right_speed)
            # max_ratio = max(abs_left, abs_right) / MAX_SPEED
            max_ratio = max(abs_left, abs_right) / self.max_linear_speed_
            if max_ratio > 1.0:  # 需要限制
                left_speed = left_speed / max_ratio
                right_speed = right_speed / max_ratio
       

            # self.send_speed(left_speed, right_speed)
            if abs(linear_velocity)<0.0001 and abs(left_speed) <= 0.0085 and abs(right_speed) <= 0.0085 and self.count_0x00 <= 4:
                self.send_speed(left_speed, right_speed)
                self.count_0x00 += 1
            elif abs(linear_velocity)<0.0001 and abs(left_speed) <= 0.0085 and abs(right_speed) <= 0.0085 and self.count_0x00 > 4:
                pass
            else:
                self.send_speed(left_speed, right_speed)
                self.count_0x00 = 0
            
        except ValueError as e:
            self.get_logger().warn(f"Invalid value: {e}")

    # def query_auto_dock_state(self):
    #     while rclpy.ok():
    #         # 发送查询auto_dock_state指令
    #         self.send_hm_auto_dock(0x01)
    #         time.sleep(0.2)


    # 处理hm_auto_dock
    def handle_hm_auto_dock(self, msg):
        # self.cmd_vel_exec_tag = False
        """处理"""
        try:
            # 消息提取
            self.get_logger().info("---handle_hm_auto_dock---")
            action_code = msg.action
            self.send_hm_auto_dock(action_code)
        except ValueError as e:
            self.get_logger().warn(f"Invalid value: {e}")

    def handle_is_near_dock(self, msg):
        """处理"""
        try:
            # 消息提取
            # self.get_logger().info("---handle_is_near_dock---")
            self.is_near_dock = msg.data
        except ValueError as e:
            self.get_logger().warn(f"Invalid value: {e}")


    ### 控制信息发送至底盘下位机
    def send_speed_approximately(self, action_value, run_time):
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
            frame += struct.pack('<H', run_time)  # 小端模式打包

            # 发送数据
            self._send_data_frame(frame)
            self.get_logger().info(f"Sent speed_approximately frame to base : 0x{action_value:02X}")

        except Exception as e:
            self.get_logger().error(f"Speed_approximately Frame sending to base failed: {str(e.message)}")
    
    def send_speed_voice_control(self, action_value):
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
            self.get_logger().info(f"Sent send_speed_voice_control frame to base : 0x{action_value:02X}")

        except Exception as e:
            self.get_logger().error(f"send_speed_voice_control Frame sending to base failed: {str(e.message)}")
    
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
            self.get_logger().error(f"Hm_auto_dock Frame sending to base failed: {str(e.message)}")

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
        # 左右轮线速度及方向
        # frame += struct.pack('<H', int(sleft*1000))
        # frame += struct.pack('<H', int(sright*1000))
        left_speed_bytes = struct.pack('<H', int(sleft*1000))
        right_speed_bytes = struct.pack('<H', int(sright*1000))
        # left_speed_bytes = b'\x43\x00'
        # right_speed_bytes = b'\x43\x00'
        frame += left_speed_bytes
        frame += right_speed_bytes
        frame += (left_dir + right_dir)

        self._send_data_frame(frame)
        # self.get_logger().info(f"Sent wheel speed frame to base: {left_speed_bytes.hex()}, {right_speed_bytes.hex()}, {left_dir.hex()}, {right_dir.hex()}")
        self.get_logger().info(f"Sent wheel speed frame to base, frame: {frame.hex()}")

    def _send_data_frame(self, data_tob_send): 
        control_data = self.control_data_head + data_tob_send + self.control_data_foot 
        try: 
            self.ser_base.write(control_data)     
        except serial.SerialTimeoutException as e:
            self.get_logger().error(f"write time out, so flush the output: {str(e.message)}")
            self.ser_base.flushOutput()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Port may disconnected, try to reopen the port. error: {str(e.message)}")
            traceback.print_exc()
            return None 
    
    ### 读取底盘下位机返回信息
    def read_base_serial_data(self):
        """读取串口数据并处理"""
        while rclpy.ok():
            # # self.get_logger().info(f"----read_base_serial_data--1--")
            # if self.ser_base.in_waiting > 0:
            #     # self.get_logger().info(f"----read_base_serial_data--2--")
            #     # 读取帧头
            #     # header = self.ser_base.read(2)
            #     header = self.ser_base.read_until(b'\xAA\x55') #会有超时时间，超时返回None

            #     if header[-2:] == b'\xAA\x55':
            #         self.get_logger().info(f"--------------read_base_serial_data--Have read header----------------")
                    
            #         ## 读取回充状态
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
            #                         msg = HMAutoDockState()
            #                         msg.state = result
            #                         self.dock_state_publisher.publish(msg)
            #                         self.get_logger().info(f"Received dock state: {result}")
            #                     else:
            #                         self.get_logger().warn("Invalid frame footer")
            #                 else:
            #                     self.get_logger().warn("Invalid command code or series number")
            #             else:
            #                 self.get_logger().warn("Incomplete data frame")

            #         ## 读取odom
            #         elif frame_length == b'\x00\x17':
            #             # self.get_logger().info("Received odom")
            #             data = self.ser_base.read(2)
            #             command_code, sequence_number= struct.unpack('BB', data)
            #             self.get_logger().info(f"Received odom, command_code:{command_code}")

            #         else:
            #             self.get_logger().warn("Invalid frame length")
            #     else:
            #         self.get_logger().warn("Invalid frame header")
            
            # time.sleep(0.005)

            data = self.get_base_new_data()
            if data is None:
                pass
                # return
            else:
                income_data_type = data.get('type')
                if income_data_type == 'odom':
                    self.cast_odom(data)
                elif income_data_type =='auto_dock':
                    self.cast_dock_state(data)
                elif income_data_type =='imu':
                    self.cast_imu(data)
                else: 
                    pass
                    return

    def get_base_new_data(self):
        try:
            frame = self.read_base_raw_frame()
            code = frame.get('code')
            data_in_frame = frame.get('data_raw')
            if code == 120: #odom
                return self._case_odom(data_in_frame)
            elif code == 39: #回充反馈
                return self._case_auto_dock(data_in_frame)
            if code == 121: #imu
                return self._case_imu(data_in_frame)
            if code == 129: #dock_ir
                return None
            if code == 130: #battery
                return None
            else:
                self.get_logger().warn(f"Unknown code_raw received: {code}")
                return None
        except BaseSerialError as e:
            self.get_logger().error(f"Read serial time out. {str(e.message)}")
            return None

    def read_base_raw_frame(self):
        frame = {}
        e = BaseSerialError()
        # header = b''
        # if self.ser_base.in_waiting > 0:
        header = self.ser_base.read_until(b'\xAA\x55')
            # header = self.ser_base.read(2)
        
        if (len(header) >= 2) and (header[-2:] != b'\xAA\x55'):
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
        if code == 39:
            data_raw =  self.ser_base.read(data_len-2) #帧长值华麦从底盘上传错误
        else:
            data_raw =  self.ser_base.read(data_len-3) #帧长-1命令码-1流水号-1帧尾

        footer = self.ser_base.read(1)
        if footer != b'\x88':
            e.message = 'No footer'
            raise e

        frame['data_raw'] = data_raw  # bytes
        frame['code'] = code  # int
        return frame
    
    @staticmethod
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
    @staticmethod
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
    @staticmethod
    def _case_imu(data_raw): 
        if len(data_raw) != 36:  # refer to the data sheet provided by jinfei
            e = BaseSerialError()
            e.message = "imu data corrupted"
            raise e
        data = {'type': 'imu'}
        acc_x_raw, acc_y_raw, acc_z_raw, gyro_x_raw, gyro_y_raw, gyro_z_raw, roll_raw, pitch_raw, yaw_raw = struct.unpack('<9f', data_raw)
        data['acc_x_raw'] = acc_x_raw
        data['acc_y_raw'] = acc_y_raw
        data['acc_z_raw'] = acc_z_raw
        data['gyro_x_raw'] = gyro_x_raw
        data['gyro_y_raw'] = gyro_y_raw
        data['gyro_z_raw'] = gyro_z_raw
        data['roll_raw'] = roll_raw
        data['pitch_raw'] = pitch_raw
        data['yaw_raw'] = yaw_raw
        return data


    def cast_odom(self, data):
        x_ori = data.get('x')
        y_ori = data.get('y')
        yaw_ori = data.get('yaw')
        v_ori = data.get('v')
        w_ori = data.get('w')
        x_ = x_ori
        y_ = y_ori
        yaw_ = yaw_ori
        # 角度归一化到 [-π, π]
        yaw_ = (yaw_ + math.pi) % (2 * math.pi) - math.pi
        v = -v_ori #下位机上传v取反
        w = w_ori #角速度不变

        #### 由于里程计上传的速度方向相反，需根据接收到的v,w重新计算odom的(x,y,yaw)
        delta_t = 0.1
        # 计算 yaw 的变化量
        delta_yaw = w * delta_t
        # 更新 yaw
        self.yaw += delta_yaw
        # 角度归一化到 [-π, π]
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        # 计算 x 和 y 的变化量
        if w != 0:
            # 如果角速度不为零，使用圆弧运动模型
            radius = v / w
            delta_x = radius * (math.sin(self.yaw + delta_yaw) - math.sin(self.yaw))
            delta_y = radius * (math.cos(self.yaw) - math.cos(self.yaw + delta_yaw))
        else:
            # 如果角速度为零，使用直线运动模型
            delta_x = v * delta_t * math.cos(self.yaw)
            delta_y = v * delta_t * math.sin(self.yaw)
        # 更新 x 和 y
        self.x += delta_x
        self.y += delta_y
        
        msg = Odometry()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0.0, 0.0, self.yaw)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        if abs(msg.twist.twist.linear.x) <= 0.001 and abs(msg.twist.twist.angular.z) <= 0.001:
            msg.twist.covariance = self.ODOM_TWIST_COVARIANCE2
            msg.pose.covariance = self.ODOM_POSE_COVARIANCE2
        else:
            msg.twist.covariance = self.ODOM_TWIST_COVARIANCE
            msg.pose.covariance = self.ODOM_POSE_COVARIANCE
        self.odom_publisher.publish(msg)

        # Publish the transform
        # yaw2 = -yaw
        # yaw2 = (yaw2 + math.pi) % (2 * math.pi) - math.pi
        quaternion2 = quaternion_from_euler(0.0, 0.0, self.yaw)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = quaternion2[0]
        transform.transform.rotation.y = quaternion2[1]
        transform.transform.rotation.z = quaternion2[2]
        transform.transform.rotation.w = quaternion2[3]

        # self.tf_broadcaster.sendTransform(transform)
    
    def cast_dock_state(self, data):
        msg = HMAutoDockState()
        msg.header.frame_id = 'auto_dock_state'
        msg.header.stamp = self.get_clock().now().to_msg()
        state = data.get('state')
        msg.state = state
        self.dock_state_publisher.publish(msg)
    
    def cast_imu(self, data):
        msg = Imu()
        msg.header.frame_id = 'imu_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.linear_acceleration.x =  data.get('acc_x_raw')
        msg.linear_acceleration.y =  data.get('acc_y_raw')
        msg.linear_acceleration.z =  data.get('acc_z_raw')
        msg.linear_acceleration_covariance = [0.000289, 0.0, 0.0, 0.0, 0.000289, 0.0, 0.0, 0.0,0.000289]
        msg.angular_velocity.x = data.get('gyro_x_raw')#*math.pi/180
        msg.angular_velocity.y = data.get('gyro_y_raw')#*math.pi/180
        msg.angular_velocity.z = data.get('gyro_z_raw')#*math.pi/180
        msg.angular_velocity_covariance = [4.0e-8, 0.0, 0.0, 0.0, 4.0e-8, 0.0, 0.0, 0.0, 4.0e-08]
        roll_t = data.get('roll_raw')#*math.pi/180
        pitch_t = data.get('pitch_raw')#*math.pi/180
        yaw_t = data.get('yaw_raw')#*math.pi/180
        quat = quaternion_from_euler(roll_t, pitch_t, yaw_t)
        msg.orientation.x =  quat[0]
        msg.orientation.y =  quat[1]
        msg.orientation.z =  quat[2]
        msg.orientation.w =  quat[3]
        msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.imu_publisher.publish(msg)

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
                                    msg0 = UInt8()
                                    msg0.data = 0
                                    for i in range(1,4):
                                        self.nav_to_goal_recycle_publisher.publish(msg0)#暂停巡航点
                                        time.sleep(0.05)
                                    time.sleep(1)
                                    msg = UInt8()
                                    msg.data = action_value
                                    for i in range(1,4):
                                        self.android_voice_motion_publisher.publish(msg)
                                        time.sleep(0.05)
                                    # self.cmd_vel_exec_tag = False
                                    # 写入android语音识别的数据到写入下位机串口
                                    # 重复写防止下位机没有反应
                                    for i in range(1,3):
                                        self.send_speed_voice_control(action_value)
                                        time.sleep(0.02)
                                    self.get_logger().info(f"Received android voice action | send_speed_voice_control: {action_value}")
                                    self.is_near_dock = 0
                                else:
                                    self.get_logger().warn("Invalid frame footer | send_speed_voice_control")
                            else:
                                self.get_logger().warn("Invalid command code or series number | send_speed_voice_control")
                        else:
                            self.get_logger().warn("Incomplete data frame | send_speed_voice_control")

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
                                    msg0 = UInt8()
                                    msg0.data = 0
                                    for i in range(1,4):
                                        self.nav_to_goal_recycle_publisher.publish(msg0)#暂停巡航点
                                        time.sleep(0.05)
                                    time.sleep(1)
                                    msg = UInt8()
                                    msg.data = command
                                    for i in range(1,4):
                                        self.android_voice_dock_publisher.publish(msg) # 调用action导航到充电桩附近的点
                                        time.sleep(0.05)
                                    # self.cmd_vel_exec_tag = False
                                    time.sleep(1)

                                    while self.is_near_dock != 1 and command == 0x02:
                                        self.get_logger().info(f"navigating to near dock: {self.is_near_dock}, {command}")
                                        time.sleep(0.2)
                                    # 写入android语音识别的数据到写入下位机串口
                                    # 重复写防止下位机没有反应
                                    for i in range(1,3):
                                        self.send_hm_auto_dock(command)
                                        time.sleep(0.2)

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
        if hasattr(self, 'ser_android') and self.ser_android.is_open:
            self.ser_android.close()

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

        # # query auto dock state
        # query_auto_dock_state_thread = threading.Thread(target=node.query_auto_dock_state)
        # query_auto_dock_state_thread.daemon = True
        # query_auto_dock_state_thread.start()

        # # 指令的连续下发
        # cmd_vel_continue_thread = threading.Thread(target=node.cmd_vel_continue)
        # cmd_vel_continue_thread.daemon = True
        # cmd_vel_continue_thread.start()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()