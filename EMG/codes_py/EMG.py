# from machine import Pin, ADC
import RPi.GPIO as GPIO
import smbus
import time

class EMG:
  def __init__(self, pin_adc):
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin_adc, GPIO.IN)
    # self.adc_pin = Pin(pin_adc, mode=Pin.IN)
    self.adc = ADC(self.adc_pin)
    self.offset = 0

  def read(self):
    reading = self.adc.read_u16() * 3.3 / 65535 - self.offset
    return reading
  
  def calibrate(self):
    print("EMG calibrating...")
    start = time.time()
    total = 0
    count = 0
    while time.time() - start < 1:
      total += self.adc.read_u16() * 3.3 / 65535
      count += 1
    self.offset = total / count