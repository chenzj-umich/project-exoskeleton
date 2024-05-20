from MPU import MPU
from Constants import I2C_ADDR

import time

mpu6050 = MPU(0, I2C_ADDR)
mpu6050.scan()
while True:
    time.sleep(50/1000)
    print(mpu6050.read_acc())

# import Constants
# 
# from machine import Pin, I2C
# 
# i2c = I2C(0, scl = Pin(1), sda = Pin(0), freq = 400000)
# 
# i2c.writeto_mem(0x68, 0x6b, bytes(1))
# word_1 = i2c.readfrom_mem(0x68, 0x3B, 1)
# print(word_1)
# high = int.from_bytes(word_1, "big") << 8
# print(high)
# word_2 = i2c.readfrom_mem(0x68, 0x3C, 1)
# print(word_2)
# low = int.from_bytes(word_2, "big")
# print(low)
# twoscomplement = b'\x80'
# result = -((65535 - high - low) + 1) if word_1 > twoscomplement else high + low
# print(result/16384*9.8)