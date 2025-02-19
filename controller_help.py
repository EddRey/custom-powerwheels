import time
from ibus import IBus
import RPi.GPIO as GPIO
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

ibus_in = IBus("/dev/serial0")

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

pca.frequency = 100

while True:
    res = ibus_in.read()
    # if signal then display immediately
    if (res[0] == 1):
        print ("Status {} CH 1 {} Ch 2 {} Ch 3 {} Ch 4 {} Ch 5 {} Ch 6 {} Ch 7 {} Ch 8{} Ch 9 {} Ch 10 {}".format(
            res[0],    # Status
            IBus.normalize(res[1]),
            IBus.normalize(res[2]),
            IBus.normalize(res[3]),
            IBus.normalize(res[4]),
            IBus.normalize(res[5], type="dial"),
            IBus.normalize(res[6], type="dial"),
            IBus.normalize(res[7]),
            IBus.normalize(res[8]),
            IBus.normalize(res[9]),
            IBus.normalize(res[10])),
            end="")
        print ("")

    else:
        print ("Status offline {}".format(res[0]))
        time.sleep(0.5)
