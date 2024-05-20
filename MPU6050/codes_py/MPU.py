#functions for MPU6050

import Constants

from Constants import ACC_SENSITIVITY as ACC_S
from Constants import GYRO_SENSITIVITY as GYRO_S

from machine import Pin, I2C

class MPU:
    def __init__(self, i2c_bus:int, i2c_addr:int):
        SDA = Pin(0) if not i2c_bus else Pin(3)
        SCL = Pin(1) if not i2c_bus else Pin(4)
        self.i2c = I2C(i2c_bus, scl = SCL, sda = SDA, freq = 400_000)
        self.i2c_addr = i2c_addr
        self.i2c.writeto_mem(i2c_addr, 0x6b, bytes(1))
        
    def scan(self):
        print('Scan i2c bus...')
        devices = self.i2c.scan()

        if len(devices) == 0:
            print("No i2c device !")
        else:
            print('i2c devices found:',len(devices))

        for device in devices:
            print("Decimal address: ",device," | Hexa address: ",hex(device))
        
    def read_acc(self):
        twoscomplement = b'\x80'
        high_X = self.i2c.readfrom_mem(self.i2c_addr, Constants.ACCEL_XOUT_H, 1)
        low_X = self.i2c.readfrom_mem(self.i2c_addr, Constants.ACCEL_XOUT_L, 1)
        acc_X =( int.from_bytes(high_X,"big") << 8 )+ int.from_bytes(low_X, "big")
        acc_X = -((65535 - acc_X) + 1) if high_X > twoscomplement else acc_X
        
        high_Y = self.i2c.readfrom_mem(self.i2c_addr, Constants.ACCEL_YOUT_H, 1)
        low_Y = self.i2c.readfrom_mem(self.i2c_addr, Constants.ACCEL_YOUT_L, 1)
        acc_Y = (int.from_bytes(high_Y, "big") << 8 )+ int.from_bytes(low_Y, "big")
        acc_Y = -((65535 - acc_Y) + 1) if high_Y > twoscomplement else acc_Y
        
        high_Z = self.i2c.readfrom_mem(self.i2c_addr, Constants.ACCEL_ZOUT_H, 1)
        low_Z = self.i2c.readfrom_mem(self.i2c_addr, Constants.ACCEL_ZOUT_L, 1)
        acc_Z = (int.from_bytes(high_Z, "big") << 8) + int.from_bytes(low_Z, "big")
        acc_Z = -((65535 - acc_Z) + 1) if high_Z > twoscomplement else acc_Z
        
        return [acc_X/ACC_S, acc_Y/ACC_S, acc_Z/ACC_S]
    
    def read_gyro(self):
        twoscomplement = b'\x80'
        high_X = self.i2c.readfrom_mem(self.i2c_addr, Constants.GYRO_XOUT_H, 1)
        low_X = self.i2c.readfrom_mem(self.i2c_addr, Constants.GYRO_XOUT_L, 1)
        gyro_X =( int.from_bytes(high_X,"big") << 8 )+ int.from_bytes(low_X, "big")
        gyro_X = -((65535 - gyro_X) + 1) if high_X > twoscomplement else gyro_X
        
        high_Y = self.i2c.readfrom_mem(self.i2c_addr, Constants.GYRO_YOUT_H, 1)
        low_Y = self.i2c.readfrom_mem(self.i2c_addr, Constants.GYRO_YOUT_L, 1)
        gyro_Y = (int.from_bytes(high_Y, "big") << 8 )+ int.from_bytes(low_Y, "big")
        gyro_Y = -((65535 - gyro_Y) + 1) if high_Y > twoscomplement else gyro_Y
        
        high_Z = self.i2c.readfrom_mem(self.i2c_addr, Constants.GYRO_ZOUT_H, 1)
        low_Z = self.i2c.readfrom_mem(self.i2c_addr, Constants.GYRO_ZOUT_L, 1)
        gyro_Z = (int.from_bytes(high_Z, "big") << 8) + int.from_bytes(low_Z, "big")
        gyro_Z = -((65535 - gyro_Z) + 1) if high_Z > twoscomplement else gyro_Z
        
        return [gyro_X/GYRO_S, gyro_Y/GYRO_S, gyro_Z/GYRO_S]