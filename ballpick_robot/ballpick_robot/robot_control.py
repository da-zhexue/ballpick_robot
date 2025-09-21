import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
import tf2_geometry_msgs
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf_transformations
import serial
import struct
import threading
import time
import math

class PIDController:
    def __init__(self, kp, ki, kd, max_output=float('inf'), min_output=-float('inf'), deadhand=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.deadband = deadhand
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

    def compute(self, setpoint, current_value):
        error = current_value - setpoint
        if abs(error) < self.deadband:
            return 0.0
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            return 0.0

        # 比例项
        proportional = self.kp * error
        # 积分项
        self.integral += error * dt
        integral_term = self.ki * self.integral
        # 微分项
        derivative = (error - self.prev_error) / dt
        derivative_term = self.kd * derivative
        self.prev_error = error

        output = proportional + integral_term + derivative_term
        output = max(self.min_output, min(self.max_output, output))
        return output

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = time.time()

class PositionController:
    def __init__(self):
        # PID控制器用于控制X位置、Y位置和偏航角(Yaw)
        self.pid_x = PIDController(kp=0.73, ki=0.01, kd=0.05, max_output=0.3, min_output=-0.3, deadhand=0.05)
        self.pid_y = PIDController(kp=14.0, ki=0.01, kd=0.10, max_output=0.8, min_output=-0.8, deadhand=0.05)
        self.pid_yaw = PIDController(kp=5.5, ki=0.008, kd=0.02, max_output=1.0, min_output=-1.0, deadhand=0.05)

    def get_control(self, current_pose, target_pose):
        """
        计算控制指令。
        :param current_pose: 小车当前位姿 (x, y, yaw)
        :param target_pose: 目标位姿 (x_target, y_target, yaw_target)
        :return: 线速度 (linear.x), 角速度 (angular.z)
        """
        current_x, current_y, current_yaw = current_pose
        target_x, target_y, target_yaw = target_pose

        # 将目标点转换到小车坐标系
        dx = target_x - current_x
        dy = target_y - current_y
        # 计算在小车坐标系下的误差
        error_x = dx * np.cos(current_yaw) + dy * np.sin(current_yaw)
        error_y = -dx * np.sin(current_yaw) + dy * np.cos(current_yaw)
        error_yaw = target_yaw - current_yaw
        # 规范化角度误差到 [-pi, pi]
        error_yaw = np.arctan2(np.sin(error_yaw), np.cos(error_yaw))
        # print(f"error_x: {error_x}, error_y: {error_y}, error_yaw: {error_yaw}")
        # 使用PID控制器计算控制量
        control_x = self.pid_x.compute(0.0, error_x)  # 目标是在小车坐标系下X误差为0
        control_y = self.pid_y.compute(0.0, error_y)  # 目标是在小车坐标系下Y误差为0
        control_yaw = self.pid_yaw.compute(0.0, error_yaw)

        # 对于阿克曼小车，Y方向的误差需要通过转向来消除，可以将其映射到角速度
        # 同时，结合X方向的控制量和Yaw方向的控制量
        linear_x = control_x  # 主要根据X方向的误差控制前进后退
        angular_z = control_yaw - control_y * 1.0  # 角速度由朝向误差和Y方向误差共同决定

        return linear_x, angular_z

class RobotControl(Node):

    def __init__(self, port=None, baudrate=115200):
        # 节点初始化
        super().__init__('robot_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10) # 创建发布者
        self.goal_subcriber_ = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10) # 创建订阅者
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self) # 创建tf监听器
        self.controller_type = "pid_position"  # "pid_position"/"stop_and_wait"
        self.target_pose = (1.0, -0.2, 0.0)  # 目标位姿 (x, y, yaw) 
        if self.controller_type == "pid_position":
            self.position_controller = PositionController()
                  
        self.tf_timer = self.create_timer(0.1, self.tf_callback) # 创建tf定时器
        self.timer = self.create_timer(0.1, self.timer_callback) # 创建定时器

        # 串口参数配置
        self.ser = None
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.stop_sent = False 
        self.need_stop = False

        # 底盘数据初始化
        self.x_speed = 0.0
        self.y_speed = 0.0
        self.z_speed = 0.0
        self.z_speed_rad = 0.0
        self.voltage = 0.0

        # 姿态数据初始化
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.reached = False
            
        # 自动检测串口设备
        if not self.port:
            self._auto_detect_port()
        # 连接串口
        if self.connect():
            self.get_logger().info("串口连接成功")
        else:
            self.get_logger().error("串口连接失败")
        
        self.target_pose_reached_pub = self.create_publisher(Bool, '/target_pose_reached', 10)

    def _auto_detect_port(self):
        """自动检测可能的USB串口设备"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'USB' in port.description or 'Serial' in port.description:
                self.port = port.device
                print(f"自动选择串口: {self.port}")
                return
        raise Exception("未找到可用串口设备")

    def connect(self):
        """连接串口设备"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,
                write_timeout=0.1
            )
            self.running = True
            # 启动接收线程
            self.recv_thread = threading.Thread(target=self._receive_data)
            self.recv_thread.daemon = True
            self.recv_thread.start()
            print(f"已连接到 {self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接并清理资源"""
        self.get_logger().info("正在断开串口连接...")
        self.running = False  # 确保接收线程退出

        if hasattr(self, 'recv_thread') and self.recv_thread.is_alive():
            try:
                self.recv_thread.join(timeout=0.5)  # 等待接收线程结束
                self.get_logger().info("接收线程已停止")
            except Exception as e:
                self.get_logger().error(f"等待接收线程时发生错误: {e}")

        # 关闭串口
        if hasattr(self, 'ser') and self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
                    self.get_logger().info("串口连接已关闭")
                else:
                    self.get_logger().info("串口已关闭，无需操作")
            except Exception as e:
                self.get_logger().error(f"关闭串口时发生错误: {e}")
            finally:
                self.ser = None

    def send_control_command(self, x_speed, y_speed, z_speed):
        """
        发送底盘控制指令
        :param x_speed: X轴目标速度 (mm/s)
        :param y_speed: Y轴目标速度 (mm/s)
        :param z_speed: Z轴目标角速度 (rad/s)，内部自动转换为mrad/s并放大1000倍
        """
        if not self.ser or not self.ser.is_open:
            print("错误: 串口未连接")
            return False

        # 构建数据包 (共11字节)
        packet = bytearray(11)
        
        # 帧头 (1字节)
        packet[0] = 0x7B
        
        # 预留位 (2字节)
        packet[1] = 0x00  # 预留位1
        packet[2] = 0x00  # 预留位2
        
        # X轴速度 (2字节，小端序)
        packet[3:5] = struct.pack('>h', int(x_speed))
        
        # Y轴速度 (2字节，小端序)
        packet[5:7] = struct.pack('>h', int(y_speed))
        
        # Z轴速度处理 (转换为mrad/s并放大1000倍)
        z_scaled = int(z_speed * 1000)  # rad/s -> mrad/s -> 放大1000倍
        packet[7:9] = struct.pack('>h', z_scaled)
        
        # 计算校验位 (前9字节异或校验)
        checksum = 0
        for i in range(9):
            checksum ^= packet[i]
        packet[9] = checksum
        
        # 帧尾 (1字节)
        packet[10] = 0x7D
        
        # 发送数据
        try:
            self.ser.write(packet)
            self.ser.flush()
            self.get_logger().info(f"已发送控制指令: X轴速度 {x_speed:.1f} mm/s, Y轴速度 {y_speed:.1f} mm/s, Z轴角速度 {z_speed:.3f} rad/s")
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            return False

    def _receive_data(self):
        """接收线程函数，持续处理串口数据"""
        buffer = bytearray()
        
        while self.running and self.ser and self.ser.is_open:
            try:
                # 读取可用数据
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    buffer.extend(data)
                    
                    # 处理完整数据帧
                    while len(buffer) >= 24:
                        # 查找帧头0x7B
                        start_idx = buffer.find(b'\x7B')
                        if start_idx == -1:
                            buffer.clear()
                            break
                            
                        # 检查是否包含完整帧 (24字节)
                        if len(buffer) < start_idx + 24:
                            break
                            
                        # 提取帧数据
                        frame = buffer[start_idx:start_idx+24]
                        del buffer[:start_idx+24]  # 移除已处理数据

                        # 验证帧尾 (索引23应为0x7D)
                        if frame[23] != 0x7D:
                            print("警告: 无效帧尾")
                            continue
                            
                        # 解析数据帧
                        self._parse_status_frame(frame)
            except Exception as e:
                print(f"接收错误: {e}")
                time.sleep(0.2)

    def _parse_status_frame(self, frame):
        """
        解析状态数据帧 (24字节)
        帧结构: [0x7B, flag, x_speed_high, x_speed_low, y_speed_high, y_speed_low, 
                z_speed_high, z_speed_low, x_accel_high, x_accel_low, y_accel_high,
                y_accel_low, z_accel_high, z_accel_low, x_angle_high, x_angle_low,
                y_angle_high, y_angle_low, z_angle_high, z_angle_low, voltage_high, 
                voltage_low, 0x7D]
        """
        try:
            # 解析速度值 (大端序，有符号short)
            self.x_speed = struct.unpack('>h', frame[2:4])[0]  # mm/s
            self.y_speed = struct.unpack('>h', frame[4:6])[0]  # mm/s
            self.z_speed = struct.unpack('>h', frame[6:8])[0]  # mrad/s
            self.z_speed_rad = self.z_speed / 1000.0

            # 解析加速度值
            # x_accel = struct.unpack('>h', frame[8:10])[0] 
            # y_accel = struct.unpack('>h', frame[10:12])[0] 
            # z_accel = struct.unpack('>h', frame[12:14])[0]  
            # 单位转换: Z轴速度 (mrad/s -> rad/s)

            # x_angle = struct.unpack('>h', frame[14:16])[0]
            # y_angle = struct.unpack('>h', frame[16:18])[0]
            # z_angle = struct.unpack('>h', frame[18:20])[0]

            self.voltage = struct.unpack('>h', frame[20:22])[0] / 1000.0  # 电压 (V)
            self.vel_publisher()
            # 输出解析结果
            # print("\n--- 底盘状态反馈 ---")
            # print(f"X轴速度: {self.x_speed} mm/s")
            # print(f"Y轴速度: {self.y_speed} mm/s")
            # print(f"Z轴角速度: {self.z_speed_rad:.3f} rad/s")
            # print(f"电池电压: {self.voltage:.2f} V")
            # 下面数据需要imu
            # print(f"X轴加速度: {x_accel}")
            # print(f"Y轴加速度: {y_accel}")
            # print(f"Z轴加速度: {z_accel}")
            # print(f"X轴角速度: {x_angle}")
            # print(f"Y轴角速度: {y_angle}")
            # print(f"Z轴角速度: {z_angle}")
            
            # print(f"当前时间: {controller.last_update_time:.2f} s")
            # print(f"当前位置: X={controller.position_x:.1f} mm, Y={controller.position_y:.1f} mm, θ={math.degrees(controller.position_theta):.1f}°")
            # print("-------------------")
        except Exception as e:
            print(f"解析错误: {e}")
    

    def vel_publisher(self):
        msg = Twist()
        # mm/s -> m/s (线性速度), rad/s 保持不变 (角速度)
        msg.linear.x = self.x_speed / 1000.0  # X轴线速度 (m/s)
        msg.linear.y = self.y_speed / 1000.0  # Y轴线速度 (m/s)
        msg.linear.z = 0.0                   # Z轴线速度 (不用)
        
        msg.angular.x = 0.0                  # X轴角速度 (不用)
        msg.angular.y = 0.0                  # Y轴角速度 (不用)
        msg.angular.z = self.z_speed_rad     # Z轴角速度 (yaw, rad/s)

        self.publisher_.publish(msg)

    def timer_callback(self):
        # 发送控制指令
        if self.running:
            self.reached = False
            if self.controller_type == "stop_and_wait":
                self.send_control_command(500, 0, 0.5)
            elif self.controller_type == "pure_pursuit":
                # 计算控制指令
                current_x = self.x / 1000.0  # 转换为米
                current_y = self.y / 1000.0  # 转换为米
                current_yaw = self.yaw
                current_pose = (current_x, current_y, current_yaw)
                
                target_linear_x, target_angular_z = self.pure_pursuit_controller.get_control(current_pose, self.target_pose)
                self.send_control_command(target_linear_x * 1000, 0, target_angular_z)
            elif self.controller_type == "pid_position":
                current_x = self.x / 1000.0  # 转换为米
                current_y = self.y / 1000.0  # 转换为米
                current_yaw = self.yaw
                current_pose = (current_x, current_y, current_yaw)  

                target_linear_x, target_angular_z = self.position_controller.get_control(current_pose, self.target_pose)
                self.send_control_command(target_linear_x * 1000, 0, target_angular_z)  # 将m/s转换为mm/s
            # 发布目标到达状态
            if abs(target_linear_x) < 1e-4:
                self.reached = True
            msg = Bool()
            msg.data = self.reached
            self.target_pose_reached_pub.publish(msg)


    def tf_callback(self):
        # 订阅tf变换
        try:
            trans = self.tf_buffer_.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.x = trans.transform.translation.x * -1000.0  # m -> mm
            self.y = trans.transform.translation.y * 1000.0  # m -> mm
            self.z = trans.transform.translation.z
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            (self.roll, self.pitch, self.yaw) = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
            self.get_logger().info(f"当前位置: X={self.x:.1f} mm, Y={self.y:.1f} mm, θ={math.degrees(self.yaw):.1f}°")
            self.get_logger().info(f"目标位置: X={self.target_pose[0]*1000:.1f} mm, Y={self.target_pose[1]*1000:.1f} mm, θ={math.degrees(self.target_pose[2]):.1f}°")
        except Exception as e:
            self.get_logger().error(f"TF错误: {e}")   
    
    def goal_pose_callback(self, msg):
        # 订阅目标位姿
        if msg.pose.position.x != 0.0 and msg.pose.position.y != 0.0:
            transform = self.tf_buffer_.lookup_transform(
                'base_link',  # 源坐标系（来自消息）
                'map',  # 目标坐标系
                rclpy.time.Time(),  # 使用最新可用的变换
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        
            # 使用 tf2_geometry_msgs 中的 do_transform_pose 进行坐标转换
            transformed_pose = tf2_geometry_msgs.do_transform_pose(msg.pose, transform)
            target_qx = transformed_pose.orientation.x
            target_qy = transformed_pose.orientation.y
            target_qz = transformed_pose.orientation.z
            target_qw = transformed_pose.orientation.w
            (target_roll, target_pitch, target_yaw) = tf_transformations.euler_from_quaternion([target_qx, target_qy, target_qz, target_qw])
            self.target_pose = (transformed_pose.position.x, transformed_pose.position.y, target_yaw)
            self.reached = False   
            self.get_logger().info(f"目标位姿: X={self.target_pose[0]:.1f} mm, Y={self.target_pose[1]:.1f} mm, θ={math.degrees(self.target_pose[2]):.1f}°")
        
    
    def send_stop_command(self):
        """发送停止指令，尝试多次"""
        max_retries = 5
        for i in range(max_retries):
            try:
                # 如果串口不可用，尝试重新连接
                if not self.ser or not self.ser.is_open:
                    self.get_logger().warn("串口未打开，尝试重新连接...")
                    if not self.connect_serial():
                        continue
                
                # 发送停止指令
                self.send_control_command(0, 0, 0.0)
                self.get_logger().info(f"第{i+1}次停止指令发送成功")
                time.sleep(0.1)  # 短暂延迟
                break
                
            except Exception as e:
                self.get_logger().error(f"第{i+1}次尝试发送停止指令失败: {e}")
                # 释放设备并尝试重新连接
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl(port="/dev/ttyACM1")
    
    try:
        rclpy.spin(robot_control)
    except KeyboardInterrupt:
        robot_control.get_logger().info("程序被键盘中断")
        robot_control.running = False
    except Exception as e:
        robot_control.get_logger().error(f"程序发生异常: {e}")
    finally:
        # 确保发送停止指令并清理资源
        robot_control.need_stop = True
        robot_control.send_stop_command()
        robot_control.disconnect()
        robot_control.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
