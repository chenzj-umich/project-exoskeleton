# from machine import Pin, ADC
import time
from EMG.codes_py.EMG import EMG
        
if __name__ == "__main__":
  emg_bi = EMG(26)
  emg_tri = EMG(27)
  
  try:
    print("initializing...1/2")
    emg_bi.calibrate()
    print("initializing...2/2")
    emg_tri.calibrate()
    print("initialization done\n")
    
    while True:
      start_time = time.clock()
      
      volt_bi = emg_bi.read()
      volt_tri = emg_tri.read()
      volt_diff = volt_bi - volt_tri
      if volt_tri == 0:
        volt_tri = 0.00001
      volt_ratio = volt_bi / volt_tri
      #print(f"EMA_bi: {round(volt_bi, 4)} EMA_tri: {round(volt_tri, 4)}")
      print(f"voltage ratio: {volt_ratio}")
      sampling_period = 0.1
      time_spent = time.time() - start_time
      time.sleep_ms(max(0, sampling_period - time_spent))
        
        
  except KeyboardInterrupt:
    print('ctrl+c pressed')
