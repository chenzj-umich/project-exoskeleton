from machine import Pin, ADC
import utime

class EMG:
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