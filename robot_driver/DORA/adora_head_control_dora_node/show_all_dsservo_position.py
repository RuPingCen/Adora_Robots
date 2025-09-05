import serial
import time
import argparse
from HeadDSServo import HeadDSServo
     
        

if __name__ == "__main__":

 
    app = HeadDSServo("serial")
     
    while True:
            print("舵机ID1当前角度 :", app.get_angle(1),"\t 舵机ID2当前角度 :", app.get_angle(2),"\n")
            time.sleep(0.02)  # 指令间隔
      
 