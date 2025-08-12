import serial
import math
import pickle
import time
import numpy as np
from MOTOR_MG4010E import MOTOR_MG4010E
 

if __name__ == "__main__":
    app = MOTOR_MG4010E(port="/dev/ttyACM0", baudrate=115200)
    time.sleep(1)
    print("motor_mg4010e_clear_multi_loop_angle id 1")
    app.motor_mg4010e_clear_multi_loop_angle(1) 
    time.sleep(1)
    print("motor_mg4010e_clear_multi_loop_angle id 2")
    app.motor_mg4010e_clear_multi_loop_angle(2) 
    time.sleep(1)
    print("motor_mg4010e_set_oringin id 1")
    app.motor_mg4010e_set_oringin(1) 
    time.sleep(1)
    print("motor_mg4010e_set_oringin id 2")
    app.motor_mg4010e_set_oringin(2) 
    time.sleep(1)
 

    while True:
        app.run()
        time.sleep(0.1)


 