#!//home/edd/.venv/bin/python3
import time
from ibus import IBus
import RPi.GPIO as GPIO
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
import pygame

ibus_in = IBus("/dev/serial0")

r_en = 27
l_en = 17

frontRight_forward = 0
frontRight_backward = 1

frontLeft_forward = 2
frontLeft_backward = 3

rearRight_forward = 4
rearRight_backward = 5

rearLeft_forward = 6
rearLeft_backward = 7

GPIO.setmode(GPIO.BCM)
GPIO.setup(r_en, GPIO.OUT)
GPIO.setup(l_en, GPIO.OUT)
GPIO.output(r_en, 1)
GPIO.output(l_en, 1)

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

controllerSpeed = .75
mSpeed = controllerSpeed * 65535
pca.frequency = 100

x = 0

firstConnection = True

soundsPath = "/home/edd/PiSounds/"

#play greeting
pygame.mixer.init()
pygame.mixer.music.set_volume(0.5) #50% volume

t = time.localtime(time.time())
print ("Hour: ", t.tm_hour)
print ("Minutes: ", t.tm_min)
print ("Seconds: ", t.tm_sec)
if (t.tm_hour >= 12):
    pygame.mixer.music.load(soundsPath + "Good_Afternoon.wav")
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
        continue
else:
    pygame.mixer.music.load(soundsPath + "Good_Morning.wav")
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy() == True:
        continueif
