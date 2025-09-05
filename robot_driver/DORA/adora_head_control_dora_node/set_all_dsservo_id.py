import serial
import time
import argparse
from HeadDSServo import HeadDSServo
 

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="设置舵机ID的脚本")
    
    # 添加位置参数
    parser.add_argument("id", type=int, help="ID")
    
    # 可选：添加更多参数
    # parser.add_argument("--option", help="可选参数示例")
    
    args = parser.parse_args()
    
    print(f"设置总线上所有ID为: {args.id}")

    app = HeadDSServo("udp")
    
    app.restore_factory_settings()
    time.sleep(2)  # 指令间隔
 
    app.set_all_motor_id(args.id)
    while True:
        print("当前角度 ID1:", app.get_angle(1))
        print("当前角度 ID2:", app.get_angle(2))
 
