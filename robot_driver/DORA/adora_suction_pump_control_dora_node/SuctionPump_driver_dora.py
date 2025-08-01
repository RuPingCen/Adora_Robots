import serial
from typing import Callable
from dora import DoraStatus
import math
import pickle
import time
import numpy as np
import pyarrow as pa
from SuctionPumpController import SuctionPumpController
 


class Operator:
    """
    打开串口读数据,校验、解析后,send_out解析得到的消息类型
    """

    # 打开串口读取数据
    def __init__(self):
        print("init SuctionPump ")
        self.app = SuctionPumpController(port="/dev/ttyUSB0", baudrate=115200)

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
         
           
        if "cmd_ch_state" == dora_input["id"]:
            #print("cmd_position event")
            cmd_ch = dora_input["value"][0].as_py() # 通道
            cmd_value = dora_input["value"][1].as_py() # 大于1打开继电器   0关闭
            #dora_input_bytes = bytes(dora_input.to_pylist())
            #self.position = pickle.loads(dora_input_bytes)
            print("set motor value: ",int(cmd_value))
            if cmd_value >= 1:
                self.app.open_channel(int(cmd_ch))
            else:
                self.app.close_channel(int(cmd_ch))

        return DoraStatus.CONTINUE
             
 
