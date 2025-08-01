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
 
    def __init__(self):
        print('init test SerialLiftingMotor node \n')
        self.counter = 0

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
        print("input event")
        self.counter = self.counter +1
        position = 0
        if self.counter <= 20:
            position = 0 
        elif self.counter <= 40 :
            position = 32768*200
        else:
            self.counter = 0
       
        send_output(
            "cmd_position",
            pa.array([position]),
            dora_input["metadata"],
        )
        return DoraStatus.CONTINUE

             