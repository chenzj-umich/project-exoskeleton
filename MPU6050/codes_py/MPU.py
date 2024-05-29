#functions for MPU6050
import os, sys
curr_dir = os.getcwd()
sys.path.insert(0, curr_dir)

import time
# from machine import Pin, I2C
import RPi.GPIO as GPIO
import smbus
import numpy as np

import MPU6050.codes_py.constants as Constants
from MPU6050.codes_py.constants import ACC_SENSITIVITY as ACC_S
from MPU6050.codes_py.constants import GYRO_SENSITIVITY as GYRO_S
# from utility import modify_constants

# from math_algo import list_div, list_add, list_sub

class MPU:
    def __init__(self, i2c_bus: int, i2c_addr: int, id):
        # Setup GPIO Pins for SDA and SCL with gpiozero if necessary
        # Note: This is typically not needed as the Pi has dedicated I2C pins and handles them with the smbus library directly
        SDA = 27 if not i2c_bus else 3  # Update these pin numbers based on your specific configuration
        SCL = 28 if not i2c_bus else 5
        # Create an SMBus instance
        self.bus = smbus.SMBus(i2c_bus)
        # I2C device address
        self.i2c_addr = i2c_addr
        # Set the MPU id (1, 2, 3...)
        self.id = id
        # Scan the bus
        self.scan()
        # Write to the device's register to initialize it
        # The register address 0x6b needs to be initialized to 0 (for example)
        self.bus.write_byte_data(self.i2c_addr, 0x6b, 0) # TODO: 0 or 1?
        # Name of the constants of MPU for the current MPU device
        self.name_acc = "OFFSET_ACC_" + str(self.id)
        self.name_att = "OFFSET_ATT_" + str(self.id)
        # Perform calibration if necessary
        self.calibrate()
        # Set the attitude relative to world-fixed frame in a rotation matrix
        self.att_mat = np.array([[0,0,0],[0,0,0],[0,0,0]])
        # Set the cumulative displacement & angle
        self.displacement = [0, 0, 0]
        self.attitude = [1, 0, 0, 0]
        
    def scan(self):
        print('Scan I2C bus...')
        # I2C addresses are usually between 0x03 and 0x77
        possible_addresses = self.i2c_addr  # Excluding reserved addresses
        found_devices = []

        try:
            # Attempt to write a quick command to the address
            # This method can throw an IOError if the device is not present
            self.bus.write_quick(possible_addresses)
            # If no error, append the device address to the list
            found_devices.append(possible_addresses)
            if not found_devices:
                print(f"MPU {self.id} not found!")
            else:
                print(f"MPU {self.id} found: Decimal address: {self.i2c_addr} | Hexa address: {hex(self.i2c_addr)}")
        except IOError:
            # No device at this address
            print("IO Error!")
        
    # TODO: read() need buffer to hold all data
    def read_acc(self):
#         twoscomplement = b'\0x80'
        twoscomplement = 128
    
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

        return [acc_X / ACC_S, acc_Y / ACC_S, acc_Z / ACC_S] # m/sec^2
    
    def read_ang_v(self):
#         twoscomplement = b'\x80'
        twoscomplement = 128

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
        gyro_Z = (high_Z << 8) | low_Z
        gyro_Z = -((65535 - gyro_Z) + 1) if high_Z > twoscomplement else gyro_Z
        
        return [np.radians(gyro_X/GYRO_S), np.radians(gyro_Y/GYRO_S), np.radians(gyro_Z/GYRO_S)] # deg/sec
    
    def calibrate(self):
        found_acc = False
        found_att = False
        with open("MPU6050/codes_py/constants.py", 'r') as file:
            lines = file.readlines()
            for line in lines:
                if line.strip().startswith(self.name_acc):
                    found_acc = True
                if line.strip().startswith(self.name_att):
                    found_att = True
        if (not found_acc) or (not found_att):
            print("MPU calibrating...")
            with open("MPU6050/codes_py/constants.py", 'w') as file:
                start = time.time()
                total_acc = np.array([0.0, 0.0, 0.0])
                total_att = np.array([0.0, 0.0, 0.0])
                count = 0
                while time.time() - start < 1:
                    total_acc += np.array(self.read_acc())
                    total_att += np.array(self.read_ang_v())
                    count += 1
                offset_acc = total_acc / count
                offset_att = total_att / count
                offset_acc_list = offset_acc.tolist()
                offset_att_list = offset_att.tolist()
                offset_acc_list[2] += -1
                # set offsets directly
                self.offset_acc = offset_acc_list
                self.offset_att = offset_att_list
                # record the offsets in constants.py
                lines.append(f"{self.name_acc} = {offset_acc_list}\n")
                lines.append(f"{self.name_att} = {offset_att_list}\n\n")
                file.writelines(lines)
            print("MPU calibrated.")
        else:
            # Set offsets from Constants
            self.offset_acc = getattr(Constants, self.name_acc, None)
            self.offset_att = getattr(Constants, self.name_att, None)
            print("MPU has already been calibrated.")

    def demo_get_displacement_attitude(self):
        start_time = time.time()
        while True:
            curr_time = time.time()
            dt = curr_time - start_time
            start_time = curr_time
            if dt < 0.05:
                print("dt < 50 ms")
                continue
            acc_list = self.read_acc()
            ang_v_list = self.read_ang_v()
            acc = np.array(acc_list)
            ang_v = np.array(ang_v_list)

            displacement = np.array(self.displacement)
            displacement += acc * dt
            self.displacement = displacement.tolist()
            attitude = np.array(self.attitude)
            attitude += ang_v * dt
            self.attitude = attitude.tolist()
            print(f"[d,a] = {self.displacement},{self.attitude}")

            sampling_period = 1 / Constants.SAMPLING_FREQUENCY
            time.sleep(max(0, sampling_period - dt))