from machine import Pin, ADC
import utime
import numpy as np


# Ports Declaration
PORT_EMG_BI = 26
PORT_EMG_TRI = 27

PORT_MPU_1_ADDR = 0x68
PORT_MPU_2_ADDR = 0x69

class ReadEMG:
  def __init__(self, pin_adc):
    self.adc_pin = Pin(pin_adc, mode=Pin.IN)
    self.adc = ADC(self.adc_pin)
    self.offset = 0

  def read(self):
    reading = self.adc.read_u16() * 3.3 / 65535 - self.offset
    return reading
  
  def calibrate(self):
    start = utime.ticks_ms()
    total = 0
    count = 0
    while utime.ticks_diff(utime.ticks_ms(), start) / 1000 < 1:
      total += self.adc.read_u16() * 3.3 / 65535
      count += 1
    self.offset = total / count

class ReadMPU:
  def __init__(self, pin_SDA, pin_SCL, ADDR):

  def read(self):
    reading = 0
    return reading


if __name__ == "__main__":
  # initialize EMGs
  emg_bi = ReadEMG(PORT_EMG_BI)
  emg_tri = ReadEMG(PORT_EMG_TRI)
  # initialize MPUs
  mpu_1 = ReadMPU()
  mpu_2 = ReadMPU()
  mpu_3 = ReadMPU()
  mpu_4 = ReadMPU()

  
  try:
    print("initializing...1/2")
    emg_bi.calibrate()
    print("initializing...2/2")
    emg_tri.calibrate()
    print("initialization done\n")
    
    while True:
      # starting time
      start_time = utime.ticks_ms()

      # read EMG
      volt_bi = emg_bi.read()
      volt_tri = emg_tri.read()
      volt_diff = volt_bi - volt_tri
      #print(f"EMA_bi: {round(volt_bi, 4)} EMA_tri: {round(volt_tri, 4)}")
      # if volt_diff < 0.01 and volt_diff > -0.01:
      #   print(f"volt_diff: 0")
      # elif volt_diff > 0:
      #   print(f"volt_diff: {volt_diff}")
      # else:
      #   print(f"volt_diff: {volt_diff}")

      # read MPU
      pos_1 = np.array(mpu_1.read())
      pos_2 = np.array(mpu_2.read())
      pos_3 = np.array(mpu_3.read())
      pos_4 = np.array(mpu_4.read())
      vec_lower = np.subtract(pos_1, pos_2)
      vec_upper = np.subtract(pos_3, pos_4)

      # delay
      sampling_period = 100
      time_spent = utime.ticks_diff(utime.ticks_ms(), start_time)
      utime.sleep_ms(max(0, sampling_period - time_spent))
        
  except KeyboardInterrupt:
    print('ctrl+c pressed')