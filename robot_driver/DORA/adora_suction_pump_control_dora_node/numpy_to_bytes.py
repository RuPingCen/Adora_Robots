import math
import pickle
import time

import numpy as np
import pyarrow as pa

import struct

# 假设这四个字节是小端序（little-endian）
byte_data = b'\x8A\x0B\x08\x00'

# 小端 int32
value = struct.unpack('<i', byte_data)[0]
print(value)  # 输出：526986



import numpy as np

# 假设你有 4 个字节，比如：
byte_data = b'\x8A\x0B\x08\x00'  # 小端序的 4 字节

# 转换为 int32
value = np.frombuffer(byte_data, dtype=np.int32)[0]

print(value)  # 输出：526986
print(type(value))  # <class 'numpy.int32'>
