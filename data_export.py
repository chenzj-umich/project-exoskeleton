# from machine import Pin, ADC
import RPi.GPIO as GPIO
import smbus
import time
import numpy as np
from MPU6050.codes_py import MPU
from EMG.codes_py import EMG
import MPU6050.codes_py.constants as Constants

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
    volts = []

    # # first-loop run
    # att_1 = np.array(mpu_1.read_att())
    # att_2 = np.array(mpu_2.read_att())
    # for mpu in mpus:
    #   # accs.append(np.array(mpu.reat_acc()))
    #   atts.append(np.array(mpu.read_att()))
    
    while True:
      # starting time
      start_time = time.time()

      # read EMG
      for emg in emgs:
        volts.append(emg.read())
      # volt_diff = volt_bi - volt_tri

      # # update MPU data
      # accs_prev = accs
      # atts_prev = atts

      # read MPU
      accs.clear()
      atts.clear()
      for mpu in mpus:
        # accs.append(np.array(mpu.reat_acc()))
        atts.append(np.array(mpu.read_att()))

      # # elbow angle calculation
      # del_att_1 = att_1 - att_1_prev
      # del_att_2 = att_2 - att_2_prev
      # del_phi_rpy = del_att_2 - del_att_1 # in [r,p,y] form


      # delay
      sampling_period = 0.1
      time_spent = time.time() - start_time
      time.sleep(max(0, sampling_period - time_spent))
        
  except KeyboardInterrupt:
    print('ctrl+c pressed')