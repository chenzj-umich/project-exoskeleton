# from machine import Pin, ADC
import RPi.GPIO as GPIO
import smbus
import time
import numpy as np
from MPU6050.codes_py import MPU
from EMG.codes_py import EMG
import MPU6050.codes_py.constants as Constants
from math_algo import rotation_matrix

# Ports Declaration
PORT_EMG_BI = 26
PORT_EMG_TRI = 27

PORT_MPU_1_ADDR = 0x68
PORT_MPU_2_ADDR = 0x69


if __name__ == "__main__":
  # initialize EMGs
  emg_bi = EMG(PORT_EMG_BI)
  emg_tri = EMG(PORT_EMG_TRI)
  emgs = [emg_bi, emg_tri]

  # initialize MPUs
  mpu_1 = MPU(0, Constants.I2C_ADDR, 1) # lower arm
  mpu_2 = MPU(0, Constants.I2C_ADDR_ALT, 2) # upper arm
  mpus = [mpu_1, mpu_2]
  
  try:
    # EMG calibration
    for i in range(len(emgs)):
      print(f"EMG calibrating...{i+1}/{len(emgs)}")
      emgs[i].calibrate()
    print("EMG calibrated.\n\n")

    accs = []
    atts = []
    ang_vs = []
    volts = []

    # TODO:
    # zero_position() for MPUs to set atts

    # # first-loop run
    # att_1 = np.array(mpu_1.read_att())
    # att_2 = np.array(mpu_2.read_att())
    # for mpu in mpus:
    #   # accs.append(np.array(mpu.reat_acc()))
    #   atts.append(np.array(mpu.read_att()))

    # starting time
    start_time = time.time()

    while True:
      # read EMG
      for emg in emgs:
        volts.append(emg.read())
      # volt_diff = volt_bi - volt_tri

      # # update MPU data
      # accs_prev = accs
      # atts_prev = atts

      # read MPU
      accs.clear()
      ang_vs.clear()
      for mpu in mpus:
        # accs.append(np.array(mpu.reat_acc()))
        ang_vs.append(np.array(mpu.read_att()))

      # inteval time
      curr_time = time.time()
      time_spent = curr_time - start_time
      start_time = curr_time

      # delta angles calculation
      for i in range(len(ang_vs)):
        del_att = rotation_matrix('world', ang_vs[i]) * time_spent
        # atts[i] += del_att
        # TODO: using matrix to update attitude: self.curr_att *= del_att


      # TODO: elbow angle calculation from rotation matrix


      # delay
      sampling_period = 1 / Constants.SAMPLING_FREQUENCY
      time.sleep(max(0, sampling_period - time_spent))
        
  except KeyboardInterrupt:
    print('ctrl+c pressed')