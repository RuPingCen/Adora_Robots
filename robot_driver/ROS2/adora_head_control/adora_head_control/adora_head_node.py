#!/usr/bin/env python3
# 节点名称：adora_head_dsservo
# 功能：舵机控制主节点框架

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, UInt8
import time
import serial
 

# 定义状态码
STATE_READY = 0
STATE_MOVING = 1
STATE_ERROR = 2

class HeadDSServo(Node):
    """舵机控制节点"""
    
    def __init__(self):
        """节点初始化"""
        self.ser = None  # 串口对象
        self.serial_opened = False  # 串口状态标志
        
         # 角度范围映射参数（根据舵机实际参数调整）
        self.ANGLE_MIN = 0
        self.ANGLE_MAX = 4095  # 对应360度
        self.DEFAULT_SPEED = 1000  # 默认转动速度（ms）

        super().__init__('adora_head_dsservo')
        
        # 声明参数
        self.declare_parameter('device_name', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        self.declare_parameter('PITCH_MIN_VALUE', 1480)  # 俯仰角范围  for pro max
        self.declare_parameter('PITCH_MAX_VALUE', 2160)  # 俯仰角范围 
        self.declare_parameter('PITCH_MID_VALUE', 1650)  # 俯仰角范围   中位

        self.declare_parameter('YAW_MIN_VALUE', 140)  # 偏航角范围 
        self.declare_parameter('YAW_MAX_VALUE', 1800)  # 偏航角范围 
        self.declare_parameter('YAW_MID_VALUE', 800)  # 偏航角范围  中位
        
        # 获取参数值
        self.device_name = self.get_parameter('device_name').value
        self.baud_rate = self.get_parameter('baud_rate').value

        self.pitch_min_value = self.get_parameter('PITCH_MIN_VALUE').value
        self.pitch_max_value = self.get_parameter('PITCH_MAX_VALUE').value
        self.pitch_mid_value = self.get_parameter('PITCH_MID_VALUE').value

        self.yaw_min_value = self.get_parameter('YAW_MIN_VALUE').value
        self.yaw_max_value = self.get_parameter('YAW_MAX_VALUE').value
        self.yaw_mid_value = self.get_parameter('YAW_MID_VALUE').value

        # 初始化日志
        self.get_logger().info('Adora Head Servo Controller Init...')
        self.get_logger().info(f'device:={self.device_name}, baud_rate={self.baud_rate}')
      
        
        # 初始化舵机位置
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # 创建订阅者 - 接收目标角度指令
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            'adora_robot/head/servo_cmd',
            self.target_callback,
            10
        )
        
        # 创建发布者 - 发布当前状态
        self.state_pub = self.create_publisher(
            Float32MultiArray,
            'adora_robot/head/head_state',
            10
        )
         

        # 初始化串口
        self.init_serial(self.device_name,self.baud_rate)
        
        self.set_angle(1, int(self.pitch_mid_value))
        self.set_angle(2, int(self.yaw_mid_value))


        # 创建周期定时器 (100ms)
        self.timer = self.create_timer(0.01, self.update_callback)
        
        # 状态变量
        self.status = STATE_READY
        
        self.get_logger().info('Adora Head Servo Controller Init Successful')
    
    def target_callback(self, msg):
        """目标角度回调函数"""
        # 验证数据格式
        if len(msg.data) < 2:
            self.get_logger().error(f'无效指令: 需要2个值, 收到{len(msg.data)}个')
            self.status = STATE_ERROR
            return
            
        pitch_rad, yaw_rad = -msg.data[0], -msg.data[1]
        self.get_logger().info(f'收到新目标位置: 俯仰={pitch_rad*57.3:.3f} 度, 水平={yaw_rad*57.3:.3f} 度')
         
        #把弧度-> 度 -> 数值
        pitch = pitch_rad*57.3*4096/360.0+self.pitch_mid_value #向下为负数  仰头为正
        yaw = yaw_rad*57.3*4096/360.0+self.yaw_mid_value #从下向上看   顺时针为负 逆时针为正
 
        if pitch < self.pitch_min_value:
            pitch = self.pitch_min_value
        elif pitch > self.pitch_max_value:
            pitch > self.pitch_max_value

        if yaw < self.yaw_min_value:
            yaw = self.yaw_min_value
        elif yaw > self.yaw_max_value:
            yaw > self.yaw_max_value


        # 设置目标位置
        self.target_pitch = pitch
        self.target_yaw = yaw
        self.status = STATE_MOVING

        self.set_angle(1, int(self.target_pitch))
        self.set_angle(2, int(self.target_yaw))
        time.sleep(0.01)  # 指令间隔
    
    def update_callback(self):
        """周期回调函数 - 处理舵机运动和发布状态"""
        # TODO: 此处添加舵机控制逻辑
        # 模拟运动过程
        move_complete = False
        if self.status == STATE_MOVING:
            # 简化处理：直接设置为目标位置
            self.current_pitch = self.target_pitch
            self.current_yaw = self.target_yaw
            self.status = STATE_READY
            move_complete = True
        
        self.current_pitch = float(self.get_angle(1))        
        self.current_yaw = float(self.get_angle(2))


        print("舵机ID1当前角度 :", self.current_pitch,"\t 舵机ID2当前角度 :", self.current_yaw,"\n")

        # 发布状态
        self.publish_state()
        
        # 如果运动完成，记录日志
        #if move_complete:
        #    self.get_logger().info(f'运动完成: 俯仰={self.current_pitch:.3f} rad, 水平={self.current_yaw:.3f} rad')
    
    def publish_state(self):
        """发布舵机状态"""
        # 创建并发布状态消息
        state_msg = Float32MultiArray()
        state_msg.data = [self.current_pitch, self.current_yaw, float(self.status)]
        self.state_pub.publish(state_msg)
        
        # 单独发布状态码
        status_msg = UInt8()
        status_msg.data = self.status
        self.status_pub.publish(status_msg)
        
        # 调试信息
        #self.get_logger().debug(
        #    f'状态发布: 俯仰={self.current_pitch:.3f}, 水平={self.current_yaw:.3f}, 状态={self.status}',
        #    throttle_duration_sec=2.0  # 限流，每2秒最多打印一次
        #)
    
    def __del__(self):
        """析构函数"""
        # TODO: 此处添加资源清理代码
        self.get_logger().info('Adora Head Servo Controller Exit...')
    def _calc_checksum(self, data):
        """统一校验和计算（带取反操作）"""
        # 计算数据包中除字头外的所有字节的和，并与0xFF进行与操作得到校验和
        # Checksum= (~ (ID 号 + 数据长度 + 指令类型+ 参数 1+ ...+ 参数 N)) & 0xFF
        return (~sum(data)) & 0xFF

    def _send_command(self, servo_id, cmd_type, params=[]):
        """发送指令通用函数"""
        # 构建数据包
        # 包括固定字头、舵机ID、数据长度、指令类型和参数
        packet = [
                0x12, 0x4C,  # 固定字头
                servo_id,  # 舵机ID
                len(params) + 2,  # 数据长度（ID + 指令类型 + 参数）
                cmd_type  # 指令类型
            ] + params

        # 计算校验和
        checksum = self._calc_checksum(packet[2:])  # 从ID开始计算
        packet.append(checksum)

        # 发送二进制数据
        self.ser.write(bytes(packet))
        time.sleep(0.01)  # 指令间隔

    def set_angle(self, servo_id, angle, runtime=None):
        """设置舵机转动到指定角度"""
        # 角度范围限制
        # 角度范围限制，确保角度在最小和最大值之间
        angle = max(self.ANGLE_MIN, min(angle, self.ANGLE_MAX))
        runtime = runtime or self.DEFAULT_SPEED  # 设置转动时间，默认为self.DEFAULT_SPEED
        # runtime：转动时间（毫秒），0表示最快速度

        # 参数打包（大端模式）将角度和时间转换为两个字节
        params = [
            0x2A,  # 目标位置地址
            (angle >> 8) & 0xFF,  # 角度高字节
            angle & 0xFF,  # 角度低字节
            (runtime >> 8) & 0xFF,  # 时间高字节
            runtime & 0xFF  # 时间低字节
        ]

        self._send_command(servo_id, 0x03, params)  # 发送设置角度的指令

    def get_angle(self, servo_id):
        """读取当前角度"""
        # 发送读指令（地址0x38，读取2字节）
        self._send_command(servo_id, 0x02, [0x38, 0x02])

        # 读取应答（等待100ms）
        response = self.ser.read(10)
        if len(response) >= 7:
            # 解析数据（大端模式）
            high = response[5]
            low = response[6]
            return (high << 8) | low
        return -1

    def set_mid_position(self, servo_id, slot=1):
        """设置中间位置（需要先手动转动到目标位置）"""
        # 地址对应关系：1->0x16, 2->0x18, 3->0x1A （对应三个目标位置）
        address_map = {1: 0x16, 2: 0x18, 3: 0x1A}
        if slot not in address_map:
            raise ValueError("Invalid slot number (1-3)")

        # 发送写入指令（无参数）设置中间位置
        self._send_command(servo_id, 0x03, [address_map[slot]])


    def set_all_motor_id(self, servo_id):
        """设置总线上所有舵机的ID 为输入servo_id"""
        # 构建数据包
        # 包括固定字头、舵机ID、数据长度、指令类型和参数
        packet = [
                0x12, 0x4C,  # 固定字头
                0xfe,  # 舵机ID
                0x04,
                0x03,
                0x05,
                servo_id  # ID
            ]

        # 计算校验和
        checksum = self._calc_checksum(packet[2:])  # 从ID开始计算
        packet.append(checksum)
        #print(bytes(packet))
        # 发送二进制数据
        self.ser.write(bytes(packet))
        time.sleep(0.01)  # 指令间隔

    def run_to_mid_position(self, servo_id, slot=1):
        """"运行到保存的中间位置"""
        # 0x3C 运行到对应位置
        self._send_command(servo_id, 0x03, [0x3C, slot])
        # 0x3C 运行保存的目标位置
        # 参数=1：运行到设定好的目标位置一
        # 参数=2：运行到设定好的目标位置二
        # 参数=3：运行到设定好的目标位置三
        # 参数=其他：无效

    def init_serial(self, port, baudrate=115200):
        try:
             # 初始化串口，设置串口参数
            self.ser = serial.Serial(
                port=port,  # 串口名称
                baudrate=baudrate,  # 波特率
                bytesize=serial.EIGHTBITS,  # 数据位8位
                parity=serial.PARITY_NONE,  # 校验位，无校验位
                stopbits=serial.STOPBITS_ONE,  # 停止位1位
                timeout=0.1  # 超时时间
        )
        except Exception as e:
            print("串口初始化失败:", e)

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        controller = HeadDSServo()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        controller.get_logger().error(f'节点异常: {str(e)}\n{traceback.format_exc()}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()