import serial
from typing import Callable
from dora import DoraStatus
import math
import pickle
import time
import numpy as np
import pyarrow as pa
from MOTOR_MG4010E import MOTOR_MG4010E
 


class Operator:
    """
    打开串口读数据,校验、解析后,send_out解析得到的消息类型
    """

    # 打开串口读取数据
    def __init__(self):
        print("init MOTOR_MG4010E ")
        self.app = MOTOR_MG4010E(port = '/dev/ttyUSB0', baudrate=115200)

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
           
        if "head_motor_angle" == dora_input["id"]:
            pitch_value = dora_input["value"][0].as_py()*57.3*10*100
            yaw_value = dora_input["value"][1].as_py()*57.3*10*100
            #dora_input_bytes = bytes(dora_input.to_pylist())
            #self.position = pickle.loads(dora_input_bytes)
            print("set pitch_value: ",int(pitch_value),"   yaw_value: ",int(yaw_value))
            self.app.motor_mg4010e_set_multi_loop_angle_control2(1,int(pitch_value),30000) # ID=1表示俯仰通道
            self.app.motor_mg4010e_set_multi_loop_angle_control2(2,int(yaw_value),30000) # ID=2表示水平旋转通道

        return DoraStatus.CONTINUE
             
 
