import time 
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)

ads = ADS.ADS1115(i2c)
ads.gain = 2/3

chan = AnalogIn(ads, ADS.P0)

while True:
	#print("{:5}\t{:5.3f}".format(chan.value, chan.voltage))
	print(int(chan.voltage/0.05))
	time.sleep(0.1)
