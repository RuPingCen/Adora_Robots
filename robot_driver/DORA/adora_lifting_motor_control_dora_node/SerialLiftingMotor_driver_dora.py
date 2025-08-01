import serial
from typing import Callable
from dora import DoraStatus
import math
import pickle
import time
import numpy as np
import pyarrow as pa
from SerialLiftingMotor import SerialLiftingMotor
 


class Operator:
    """
    打开串口读数据,校验、解析后,send_out解析得到的消息类型
    """

    # 打开串口读取数据
    def __init__(self):
        print("init SerialLiftingMotor ")
        self.app = SerialLiftingMotor()

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
        if "tick" == dora_input["id"]:
            print("tick event")
            self.app.motor_position_read()
            self.app.run()
            send_output(
                "cur_position",
                pa.array([self.app.motor_positon_read]),
                dora_input["metadata"],
            )
           
        if "cmd_position" == dora_input["id"]:
            #print("cmd_position event")
            motor_value = dora_input["value"][0].as_py()
            #dora_input_bytes = bytes(dora_input.to_pylist())
            #self.position = pickle.loads(dora_input_bytes)
            print("set motor value: ",int(motor_value))
            self.app.cmd_vel_callback(int(motor_value))

        return DoraStatus.CONTINUE
             
 
