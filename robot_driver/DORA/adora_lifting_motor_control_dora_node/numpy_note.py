import math
import pickle
import time
import numpy as np
import pyarrow as pa


# 测试float类型数据到int32类型数据转换

 
# 传入浮点数
float_value = -123.132324123
#print("float_value: hex    ",hex(float_value))

# 浮点数到int32类型数据转换
int32_value = np.int32(float_value)
print("int32_value:  %d“,  hex    ",int32_value,hex(int32_value))

 
data_array_1 = bytearray()
data_array_1.extend([0x00, 0x00, 0x00, 0x00])  # set motor position
data_array_1[0] = (int32_value>>24) & 0xff
data_array_1[1] = (int32_value>>16) & 0xff
data_array_1[2] = (int32_value>>8) & 0xff
data_array_1[3] = int32_value & 0xff
print(f"CRC16: 0x{data_array_1.hex().upper()}")


# 将四个四字节数据合成一个int32类型数据


int32_num = (data_array_1[0] << 24) | (data_array_1[1] << 16) | (data_array_1[2] << 8) | data_array_1[3]
if int32_num & 0x80000000:
    int32_num -= 0x100000000  # 转为负数（有符号）
print("四个四字节数据合成一个int32类型数据:  num:  ",int32_num,"   hex",hex(int32_num))
 
print("int(hex_value, 8)  :  ",int(int32_num, 8))
 