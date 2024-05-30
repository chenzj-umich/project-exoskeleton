import os, sys
curr_dir = os.getcwd()
sys.path.insert(0, curr_dir)

from MPU6050.codes_py.MPU import MPU
from MPU6050.codes_py.constants import I2C_ADDR

import time

import numpy as np

mpu1 = MPU(1, I2C_ADDR, 1)
# import MPU6050.codes_py.constants as Constant

print(mpu1.offset_acc)
# print(mpu6050.read_acc(), mpu6050.read_ang_v())

print("MPU calibration done\n")


# acc_threshold = 0.01
# att_threshold = 0.01

# while True:
#     time.sleep(50/1000)
#     if mpu1.offset_acc is not None:
#         acc = (np.array(mpu1.read_acc()) - np.array(mpu1.offset_acc)).tolist()
#         att = (np.array(mpu1.read_ang_v()) - np.array(mpu1.offset_att)).tolist()
#         s = 0
#         for num in acc:
#             s += num * num
#         if s - 1 < acc_threshold and s - 1 > -1 * acc_threshold:
#             acc = [0, 0, 0]
#         for num in att:
#             if num < att_threshold:
#                 num = 0.0
#         print([round(num,4) for num in acc], [round(num,4) for num in att])
#     else:
#         print("offsets None.")

# mpu1.demo_get_displacement_attitude()