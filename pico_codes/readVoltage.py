from machine import Pin, ADC
import time


class ReadIn:
    def __init__(self, pin_adc, alpha, size):
        self.adc_pin = Pin(pin_adc, mode=Pin.IN)
        self.adc = ADC(self.adc_pin)
#         # Exponential Moving Average
#         self.alpha = alpha
#         self.ema = 0
#         # General Average
#         self.buffer_size = size
#         self.readings = [0] * size
#         self.readings_ema = [0] * size
#         self.index = 0
        self.offset = 0

    def read(self):
        new_reading = self.adc.read_u16() * 3.3 / 65535 - self.offset
#         self.ema = self.alpha * new_reading + (1 - self.alpha) * self.ema
#         if new_reading < 0.15:
#             new_reading = 0
#         if self.ema < 0.15:
#             self.ema = 0
#         self.readings[self.index] = new_reading
#         self.readings_ema[self.index] = self.ema
#         self.index = (self.index + 1) % self.buffer_size
#         
#         average = sum(self.readings) / self.buffer_size
#         average_ema = sum(self.readings_ema) / self.buffer_size
#         voltage = average
#         voltage_ema = average_ema
        return new_reading
        
    
    def calibrate(self):
        start = time.ticks_ms()
        total = 0
        count = 0
        while time.ticks_diff(time.ticks_ms(), start) / 1000 < 1:
            total += self.adc.read_u16() * 3.3 / 65535
            count += 1
        self.offset = total / count
        #print(self.offset)
        
if __name__ == "__main__":
    emg_bi = ReadIn(26, 0.5, 10)
    emg_tri = ReadIn(27, 0.5, 10)
    
    try:
        print("initializing...1/2")
        emg_bi.calibrate()
        print("initializing...2/2")
        emg_tri.calibrate()
        print("initialization done\n")
        
        while True:
            start_time = time.ticks_ms()
            
            volt_bi = emg_bi.read()
            volt_tri = emg_tri.read()
            volt_diff = volt_bi - volt_tri
            #print(f"EMA_bi: {round(volt_bi, 4)} EMA_tri: {round(volt_tri, 4)}")
            if volt_diff < 0.01 and volt_diff > -0.01:
                print(f"volt_diff: 0")
            elif volt_diff > 0:
                print(f"volt_diff: {volt_diff}")
            else:
                print(f"volt_diff: {volt_diff}")
            
            sampling_period = 100
            time_spent = time.ticks_diff(time.ticks_ms(), start_time)
            time.sleep_ms(max(0, sampling_period - time_spent))
        
        
    except KeyboardInterrupt:
        print('ctrl+c pressed')