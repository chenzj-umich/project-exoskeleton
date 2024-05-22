#functions for MPU6050

import time
# from machine import Pin, I2C
import RPi.GPIO as GPIO
import smbus

import MPU6050.codes_py.constants as Constants
from MPU6050.codes_py.constants import ACC_SENSITIVITY as ACC_S
from MPU6050.codes_py.constants import GYRO_SENSITIVITY as GYRO_S
from utility import modify_constants

from math_algo import list_div, list_add, list_sub

class MPU:
    def __init__(self, i2c_bus: int, i2c_addr: int):
        # Setup GPIO Pins for SDA and SCL with gpiozero if necessary
        # Note: This is typically not needed as the Pi has dedicated I2C pins and handles them with the smbus library directly
        SDA = 27 if not i2c_bus else 3  # Update these pin numbers based on your specific configuration
        SCL = 28 if not i2c_bus else 5
        # Create an SMBus instance
        self.bus = smbus.SMBus(i2c_bus)
        # I2C device address
        self.i2c_addr = i2c_addr
        # Write to the device's register to initialize it
        # The register address 0x6b needs to be initialized to 0 (for example)
        self.bus.write_byte_data(self.i2c_addr, 0x6b, 0) # TODO: 0 or 1?
        # Perform calibration if necessary
        self.calibrate()
        # Set offsets from Constants
        self.offset_acc = OFFSET_ACC
        self.offset_att = OFFSET_ATT
        
    def scan(self):
        print('Scan I2C bus...')
        # I2C addresses are usually between 0x03 and 0x77
        possible_addresses = range(0x03, 0x78)  # Excluding reserved addresses
        found_devices = []

        for address in possible_addresses:
            try:
                # Attempt to write a quick command to the address
                # This method can throw an IOError if the device is not present
                self.bus.write_quick(address)
                # If no error, append the device address to the list
                found_devices.append(address)
            except IOError:
                # No device at this address
                continue

        if not found_devices:
            print("No I2C devices found!")
        else:
            print('I2C devices found:', len(found_devices))
            for device in found_devices:
                print("Decimal address:", device, " | Hexa address:", hex(device))

        
    def read_acc(self):
        twoscomplement = b'\0x80'
    
        high_X = self.bus.read_byte_data(self.i2c_addr, Constants.ACCEL_XOUT_H)
        low_X = self.bus.read_byte_data(self.i2c_addr, Constants.ACCEL_XOUT_L)
        acc_X = (high_X << 8) | low_X
        acc_X = -((65535 - acc_X) + 1) if high_X > twoscomplement else acc_X

        high_Y = self.bus.read_byte_data(self.i2c_addr, Constants.ACCEL_YOUT_H)
        low_Y = self.bus.read_byte_data(self.i2c_addr, Constants.ACCEL_YOUT_L)
        acc_Y = (high_Y << 8) | low_Y
        acc_Y = -((65535 - acc_Y) + 1) if high_Y > twoscomplement else acc_Y

        high_Z = self.bus.read_byte_data(self.i2c_addr, Constants.ACCEL_ZOUT_H)
        low_Z = self.bus.read_byte_data(self.i2c_addr, Constants.ACCEL_ZOUT_L)
        acc_Z = (high_Z << 8) | low_Z
        acc_Z = -((65535 - acc_Z) + 1) if high_Z > twoscomplement else acc_Z

        return [acc_X / ACC_S, acc_Y / ACC_S, acc_Z / ACC_S]
    
    def read_att(self):
        twoscomplement = b'\x80'

        high_X = self.bus.read_byte_data(self.i2c_addr, Constants.GYRO_XOUT_H)
        low_X = self.bus.read_byte_data(self.i2c_addr, Constants.GYRO_XOUT_L)
        gyro_X = (high_X << 8) | low_X
        gyro_X = -((65535 - gyro_X) + 1) if high_X > twoscomplement else gyro_X
        
        high_Y = self.bus.read_byte_data(self.i2c_addr, Constants.GYRO_YOUT_H)
        low_Y = self.bus.read_byte_data(self.i2c_addr, Constants.GYRO_YOUT_L)
        gyro_Y = (high_Y << 8 ) | low_Y
        gyro_Y = -((65535 - gyro_Y) + 1) if high_Y > twoscomplement else gyro_Y
        
        high_Z = self.bus.read_byte_data(self.i2c_addr, Constants.GYRO_ZOUT_H)
        low_Z = self.bus.read_byte_data(self.i2c_addr, Constants.GYRO_ZOUT_L)
        gyro_Z = (high_Z<< 8) | low_Z
        gyro_Z = -((65535 - gyro_Z) + 1) if high_Z > twoscomplement else gyro_Z
        
        return [gyro_X/GYRO_S, gyro_Y/GYRO_S, gyro_Z/GYRO_S]
    
    def calibrate(self):
        print("MPU calibrating...")
        start = time.ticks_ms()
        total_acc = [0, 0, 0]
        total_att = [0, 0, 0]
        count = 0
        while time.time() - start < 1:
            total_acc = list_add(total_acc, self.read_acc())
            total_att = list_add(total_att, self.read_att())
            count += 1
        offset_acc = list_div(total_acc, count)
        offset_acc[2] += -1
        offset_att = list_div(total_att, count)
        # modify the constants
        modify_constants("MPU6050/codes_py/constants.py", "OFFSET_ACC", offset_acc)
        modify_constants("MPU6050/codes_py/constants.py", "OFFSET_ATT", offset_att)