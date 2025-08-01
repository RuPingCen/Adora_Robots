import serial
import time
import argparse
 

class SerialSliderApp:
    def __init__(self):
        self.ser = None  # 串口对象
        self.serial_opened = False  # 串口状态标志
         
        # 初始化串口
        self.init_serial("/dev/ttyUSB0",115200)

        # 角度范围映射参数（根据舵机实际参数调整）
        self.ANGLE_MIN = 0
        self.ANGLE_MAX = 4095  # 对应360度
        self.DEFAULT_SPEED = 1000  # 默认转动速度（ms）


         
        # """设置中间位置："""
        self.set_mid_position(1, slot=1)
        # 3071 为上方舵机的中值
        # 上方舵机应从2071.5~4095，限制在180~360之间，下方舵机应从0~4095，限制在0~360之间

        """设置目标角度（上方舵机）："""
        # goal_angle = input("输入目标角度：")
        # goal_angle = int(goal_angle)
        # goal_angle = (goal_angle * 2047.5/180)+2047.5
        # controller.set_angle(1, int(goal_angle))
        # print("当前角度：", (controller.get_angle(1)-2047.5)*180/2047.5)

        """运行到保存的中间位置："""
        #controller.run_to_mid_position(1, slot=1)
 
 
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

    def restore_factory_settings(self):
        """恢复出厂设置"""
        # 构建数据包
        # 包括固定字头、舵机ID、数据长度、指令类型和参数
        packet = [
                0x12, 0x4C,  # 固定字头
                0x01,  # 舵机ID
                0x02,
                0x06,
                0xF6]
        # 发送二进制数据
        self.ser.write(bytes(packet))
        time.sleep(1)  # 指令间隔

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

 

 
    def run(self,target_ID):
         
        app.set_all_motor_id(target_ID)
        while True:
            print("当前角度 ID1:", self.get_angle(1))
            print("当前角度 ID2:", self.get_angle(2))
            
        if self.ser and self.ser.is_open:
                self.ser.close()    

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="设置舵机ID的脚本")
    
    # 添加位置参数
    parser.add_argument("id", type=int, help="ID")
    
    # 可选：添加更多参数
    # parser.add_argument("--option", help="可选参数示例")
    
    args = parser.parse_args()
    
    print(f"设置总线上所有ID为: {args.id}")

    app = SerialSliderApp()
    
    app.restore_factory_settings()
    time.sleep(2)  # 指令间隔
    app.run(args.id)
 
