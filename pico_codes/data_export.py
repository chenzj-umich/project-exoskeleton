from machine import Pin, ADC
import utime
import numpy as np
from MPU6050.codes_py import MPU
from EMG.codes_py import EMG

# Ports Declaration
PORT_EMG_BI = 26
PORT_EMG_TRI = 27

PORT_MPU_1_ADDR = 0x68
PORT_MPU_2_ADDR = 0x69


if __name__ == "__main__":
  # initialize EMGs
  emg_bi = EMG(PORT_EMG_BI)
  emg_tri = EMG(PORT_EMG_TRI)

  # initialize MPUs
  # TODO
  mpu_1 = MPU(0, I2C_ADDR) # lower arm
  mpu_2 = MPU(0, I2C_ADDR_ALT) # upper arm

  
  try:
    # EMG calibration
    emg_bi.calibrate()
    emg_tri.calibrate()
    print("EMG calibration done\n")

    # MPU calibration
    mpu_1.calibrate()
    mpu_2.calibrate()
    print("MPU calibration done\n")

    # first-loop run
    att_1 = np.array(mpu_1.read_att())
    att_2 = np.array(mpu_2.read_att())
    
    while True:
      # starting time
      start_time = utime.ticks_ms()

      # read EMG
      volt_bi = emg_bi.read()
      volt_tri = emg_tri.read()
      volt_diff = volt_bi - volt_tri
      #print(f"EMA_bi: {round(volt_bi, 4)} EMA_tri: {round(volt_tri, 4)}")

      # update MPU data
      att_1_prev = att_1
      att_2_prev = att_2

      # read MPU
      att_1 = np.array(mpu_1.read_att())
      att_2 = np.array(mpu_2.read_att())

      # elbow angle calculation
      del_att_1 = att_1 - att_1_prev
      del_att_2 = att_2 - att_2_prev
      del_phi_rpy = del_att_2 - del_att_1 # in [r,p,y] form


      # delay
      sampling_period = 100
      time_spent = utime.ticks_diff(utime.ticks_ms(), start_time)
      utime.sleep_ms(max(0, sampling_period - time_spent))
        
  except KeyboardInterrupt:
    print('ctrl+c pressed')