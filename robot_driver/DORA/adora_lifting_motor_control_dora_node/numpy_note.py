import math
import pickle
import time
import numpy as np
import pyarrow as pa


# 测试float类型数据到int32类型数据转换

  

# 传入浮点数

 

float64_value = 3.141592653589793  # float64
float32_value = np.float32(float64_value)  # 输出：3.1415927（末尾截断）

print("float64_value: ",float64_value)
print("float32_value: ",float32_value)


data_array_0 = float32_value.tobytes()  # 返回4字节的bytes对象
print(data_array_0)  # 示例输出：b'\xdb\x0fI@'
print(len(data_array_0))  # 输出：4（确认字节长度）
print(f"float32_value CRC16: 0x{data_array_0.hex().upper()}")


float_num = np.frombuffer(data_array_0, dtype=np.float32)[0]
print("float32_value frombuffer:" ,float_num)  # 输出：3.14


double_value = -1232847923.132324123
# 浮点数到int32类型数据转换 占4个字节
int32_value = np.int32(double_value)
print("int32_value:  %d“,  hex    ",int32_value,hex(int32_value))

 
data_array_1 = bytearray()
data_array_1.extend([0x00, 0x00, 0x00, 0x00])  # set motor position
data_array_1[0] = (int32_value>>24) & 0xff
data_array_1[1] = (int32_value>>16) & 0xff
data_array_1[2] = (int32_value>>8) & 0xff
data_array_1[3] = int32_value & 0xff
print(f"int32_value CRC16: 0x{data_array_1.hex().upper()}")


# 将四个四字节数据合成一个int32类型数据
#方法1：
int32_num = (data_array_1[0] << 24) | (data_array_1[1] << 16) | (data_array_1[2] << 8) | data_array_1[3]
if int32_num & 0x80000000:
    int32_num -= 0x100000000  # 转为负数（有符号）
print("#方法1: 四个四字节数据合成一个int32类型数据:  num:  ",int32_num,"   hex",hex(int32_num))
#方法2：
position_bytes = bytearray()
position_bytes.extend([data_array_1[0], data_array_1[1],data_array_1[2], data_array_1[3]]) #高位存低地址
int32_num2 = int.from_bytes(position_bytes,'big',signed = True)
print("#方法2:   ",int32_num2)
 