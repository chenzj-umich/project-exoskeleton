from MPU6050.codes_py.MPU import MPU
from MPU6050.codes_py.constants import I2C_ADDR

import time

from math_algo import list_div, list_add, list_sub

mpu6050 = MPU(0, I2C_ADDR)
mpu6050.scan()
print(mpu6050.read_acc(), mpu6050.read_ang_v())

print("MPU calibration done\n")

while True:
    time.sleep(50/1000)
    
    print(list_sub(mpu6050.read_acc(), mpu6050.offset_acc), list_sub(mpu6050.read_ang_v(), mpu6050.offset_att))