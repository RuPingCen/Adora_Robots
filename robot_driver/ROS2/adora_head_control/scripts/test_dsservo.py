import serial
import time

import tkinter as tk
import tkinter as tk
import serial

class SerialSliderApp:
    def __init__(self):
        self.ser = None  # 串口对象
        self.serial_opened = False  # 串口状态标志
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("重邮串口控制台")
        self.root.geometry("500x400")

        # 标题标签
        title_label = tk.Label(self.root, 
                              text="重庆邮电大学\n Adora机器人头部舵机调试系统",
                              font=("微软雅黑", 18),
                              fg="blue")
        title_label.pack(pady=15)
      
        # 创建滑动条
        self.create_sliders()
        
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

      
 
    def create_sliders(self):
        # 滑动条1
        self.slider1 = tk.Scale(self.root,
                               from_=0, to=180,
                               orient=tk.HORIZONTAL,
                               label="俯仰(ID = 1)",
                               length=300,
                               command=self.send_values)
        self.slider1.pack(pady=10)

        # 滑动条2
        self.slider2 = tk.Scale(self.root,
                               from_=0, to=360,
                               orient=tk.HORIZONTAL,
                               label="旋转(ID = 2)",
                               length=300,
                               command=self.send_values)
        self.slider2.pack(pady=10)

    
    def send_values(self, value):
        """滑动条数值变化时触发"""
        # 终端打印数值
        print(f"通道1: {self.slider1.get()} | 通道2: {self.slider2.get()}")
        
        # 通过串口发送数据
        #goal_angle = input("输入目标角度：")
        goal_angle = int(self.slider1.get())
        goal_angle = (goal_angle * 2047.5/180)+2047.5
        self.set_angle(1, int(goal_angle))


        goal_angle = int(self.slider2.get())
        goal_angle = (goal_angle * 2047.5/180)+2047.5
        self.set_angle(2, int(goal_angle))

        print("舵机ID1当前角度 :", self.get_angle(1),"\t 舵机ID2当前角度 :", self.get_angle(2),"\n")


    def draw_led(self, color):
        """绘制状态指示灯"""
        self.status_led.delete("all")
        self.status_led.create_oval(2,2,18,18,
                                   fill=color,
                                   outline="black")

    def run(self):
        self.root.mainloop()
        # 窗口关闭时确保串口关闭
        if self.ser and self.ser.is_open:
            self.ser.close()

if __name__ == "__main__":
    app = SerialSliderApp()
    app.run()
 
