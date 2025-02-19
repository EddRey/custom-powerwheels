#!//home/edd/.venv/bin/python3
import multiprocessing
import time
import board
from board import SCL, SDA
import busio
from ibus import IBus
import RPi.GPIO as GPIO
from pcf8575 import PCF8575
from adafruit_pca9685 import PCA9685
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import pygame
import sys

ibus_in = IBus("/dev/serial0")

pcf_address = 0x20
i2c_port_num = 1
pcf = PCF8575(i2c_port_num, pcf_address)

i2c_bus = busio.I2C(SCL, SDA)

#Front Right motor
FR_en = 24

#Front left motor
FL_en = 18

#Rear Right motor
RR_en = 17

#Rear Left motor
RL_en = 25

#Steering motor
S_en = 23

#PWM board connections
frontRight_forward = 0
frontRight_backward = 1

frontLeft_forward = 2
frontLeft_backward = 3

rearLeft_forward = 5
rearLeft_backward = 4

rearRight_forward = 7
rearRight_backward = 6

steering_forward = 8
steering_backward = 9

GPIO.setmode(GPIO.BCM)

#setup pins
GPIO.setup(FR_en, GPIO.OUT)

GPIO.setup(FL_en, GPIO.OUT)

GPIO.setup(RR_en, GPIO.OUT)

GPIO.setup(RL_en, GPIO.OUT)

GPIO.setup(S_en, GPIO.OUT)

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)

pca.frequency = 100

#extended GPIO pins set up (PCF)
i2c_port_num = 1

#potentiometer setup
ads = ADS.ADS1115(i2c_bus)
ads.gain = 2/3

potenio = AnalogIn(ads, ADS.P0)

firstConnection = True

currentMode = 0
direction = " "
speed = 0

FWenable = 1
RWenable = 1

controller = 100
manual = -100

#only use a max of 98% motorspeed
maxSpeed = 0.98 * 65535
speedUpRate = 5
slowDownRate = 1

swapModes = True

soundsPath = "/home/edd/PiSounds/"

def status(mutex):
    global currentMode

    res = [0,0,0,0,0,0,0,0,0,0]

    pygame.mixer.init()
    pygame.mixer.music.set_volume(1) #850% volume

    t = time.localtime(time.time())

    if (t.tm_hour >= 12):
        pygame.mixer.music.load(soundsPath + "Good_Afternoon.wav")
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy() == True:
                continue
    else:
        pygame.mixer.music.load(soundsPath + "Good_Morning.wav")
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy() == True:
                continue
    while True:
        #steering wheel buttons
        RWheel = pcf.port[4] #horn
        LWheel = pcf.port[6] #chirp chirp

        if(RWheel == False):
                        pygame.mixer.music.load(soundsPath + "carhorn.mp3")
                        pygame.mixer.music.play()
                        while pygame.mixer.music.get_busy() == True:
                                continue

        elif(LWheel == False):
                pygame.mixer.music.load(soundsPath + "carlock.mp3")
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy() == True:
                        continue
        try:
                mutex.acquire()
                res = ibus_in.read()
                mutex.release()
        except:
                print("Error with Ibus in status(). Trying again.")
                pass

        if (IBus.normalize(res[10]) == -100):
                mode = IBus.normalize(res[7])
                if (mode == manual and currentMode != mode):
                        pygame.mixer.music.load(soundsPath + "Manual_Mode.wav")
                        pygame.mixer.music.play()
                        while pygame.mixer.music.get_busy() == True:
                                continue
                elif (mode == controller and currentMode != mode):
                        pygame.mixer.music.load(soundsPath + "Controller_mode.wav")
                        pygame.mixer.music.play()
                        while pygame.mixer.music.get_busy() == True:
                                continue
                currentMode = mode


def main(mutex):
    global currentMode
    global direction
    global speed

    res = [0,0,0,0,0,0,0,0,0,0]

    while True:
        try:
                mutex.acquire()
                res = ibus_in.read()
                mutex.release()
        except:
                print("Error with Ibus in main(). Trying again.")
                pass

        if(res[0] == 1):
            if(IBus.normalize(res[10]) == -100):
                mode = IBus.normalize(res[7])

                #manual mode
                if (mode == manual):

                        #onboard switches
                        #beans = pcf.port[5]
                        pedal = pcf.port[7]
                        fourWheel = pcf.port[9]
                        twoWheel = pcf.port[10]
                        forward = pcf.port[11]
                        backward = pcf.port[12]

                        if(fourWheel == False):
                                FWenable = 1
                                RWenable = 1
                        elif(twoWheel == False):
                                FWenable = 0
                                RWenable = 1

                        GPIO.output(S_en, 0)
                        #direction
                        if(forward == True and backward == True):
                                #disable motors (neutral)
                                GPIO.output(FR_en, 0)

                                GPIO.output(FL_en, 0)

                                GPIO.output(RR_en, 0)

                                GPIO.output(RL_en, 0)
                        elif(forward == False and backward == True):
                                if(direction == "forward"):
                                        #go forward
                                        GPIO.output(FR_en, FWenable)

                                        GPIO.output(FL_en, FWenable)

                                        GPIO.output(RR_en, RWenable)

                                        GPIO.output(RL_en, RWenable)
                                else:
                                        GPIO.output(FR_en, 0)

                                        GPIO.output(FL_en, 0)

                                        GPIO.output(RR_en, 0)

                                        GPIO.output(RL_en, 0)

                                        time.sleep(0.1)

                                        #go forward
                                        GPIO.output(FR_en, FWenable)

                                        GPIO.output(FL_en, FWenable)

                                        GPIO.output(RR_en, RWenable)

                                        GPIO.output(RL_en, RWenable)
                                        direction = "forward"

                                pValue = int(potenio.voltage/0.05) #potentiometerValue
                                #speed up
                                if (pedal == False):
                                        speed = int((pValue/100) * maxSpeed)
                                        pca.channels[frontRight_forward].duty_cycle = speed
                                        pca.channels[frontLeft_forward].duty_cycle = speed
                                        pca.channels[rearRight_forward].duty_cycle = speed
                                        pca.channels[rearLeft_forward].duty_cycle = speed
                                        pca.channels[frontRight_backward].duty_cycle = 0
                                        pca.channels[frontLeft_backward].duty_cycle = 0
                                        pca.channels[rearRight_backward].duty_cycle = 0
                                        pca.channels[rearLeft_backward].duty_cycle = 0
                                #slow down
                                else:
                                        #stop
                                        pca.channels[frontRight_forward].duty_cycle = 0
                                        pca.channels[frontLeft_forward].duty_cycle = 0
                                        pca.channels[rearRight_forward].duty_cycle = 0
                                        pca.channels[rearLeft_forward].duty_cycle = 0
                                        pca.channels[frontRight_backward].duty_cycle = 0
                                        pca.channels[frontLeft_backward].duty_cycle = 0
                                        pca.channels[rearRight_backward].duty_cycle = 0
                                        pca.channels[rearLeft_backward].duty_cycle = 0
                        #go backward
                        elif(forward == True and backward == False):
                                if(direction == "backward"):
                                        #go backward
                                        GPIO.output(FR_en, FWenable)

                                        GPIO.output(FL_en, FWenable)

                                        GPIO.output(RR_en, RWenable)

                                        GPIO.output(RL_en, RWenable)
                                else:
                                        GPIO.output(FR_en, 0)

                                        GPIO.output(FL_en, 0)

                                        GPIO.output(RR_en, 0)

                                        GPIO.output(RL_en, 0)

                                        time.sleep(0.1)

                                        #go backward
                                        GPIO.output(FR_en, FWenable)

                                        GPIO.output(FL_en, FWenable)

                                        GPIO.output(RR_en, RWenable)

                                        GPIO.output(RL_en, RWenable)
                                        direction = "backward"

                                #speed up
                                pValue = int(potenio.voltage/0.05) #potentiometerValue
                                if (pedal == False):
                                        speed = int((pValue/100) * maxSpeed)
                                        pca.channels[frontRight_backward].duty_cycle = speed
                                        pca.channels[frontLeft_backward].duty_cycle = speed
                                        pca.channels[rearRight_backward].duty_cycle = speed
                                        pca.channels[rearLeft_backward].duty_cycle = speed
                                        pca.channels[frontRight_forward].duty_cycle = 0
                                        pca.channels[frontLeft_forward].duty_cycle = 0
                                        pca.channels[rearRight_forward].duty_cycle = 0
                                        pca.channels[rearLeft_forward].duty_cycle = 0
                                #slow down
                                else:
                                        pca.channels[frontRight_backward].duty_cycle = 0
                                        pca.channels[frontLeft_backward].duty_cycle = 0
                                        pca.channels[rearRight_backward].duty_cycle = 0
                                        pca.channels[rearLeft_backward].duty_cycle = 0
                                        pca.channels[frontRight_forward].duty_cycle = 0
                                        pca.channels[frontLeft_forward].duty_cycle = 0
                                        pca.channels[rearRight_forward].duty_cycle = 0
                                        pca.channels[rearLeft_forward].duty_cycle = 0
                #controller mode
                elif (mode == controller):
                        GPIO.output(S_en, 0) #steering motor is always enabled in contrller mode
                        GPIO.output(FR_en, 0)
                        GPIO.output(FL_en, 0)
                        GPIO.output(RR_en, 0)
                        GPIO.output(RL_en, 0)

                        if(IBus.normalize(res[9]) == -100):
                                FWenable = 1
                                RWenable = 1
                        elif(IBus.normalize(res[9]) == 100):
                                FWenable = 0
                                RWenable = 1
                        elif(IBus.normalize(res[9]) == 0):
                                FWenable = 0
                                RWenable = 0
                        GPIO.output(S_en, 1) #steering motor is always enabled in contrller mode

                        GPIO.output(FR_en, FWenable)

                        GPIO.output(FL_en, FWenable)

                        GPIO.output(RR_en, RWenable)

                        GPIO.output(RL_en, RWenable)

                        poten = (IBus.normalize(res[6]) +100)/2

                        #forward control
                        if(int(maxSpeed * (IBus.normalize(res[2])/100)) > 0.5):
                                #go forward
                                motorSpeed = abs(int(maxSpeed * ((poten/100) * (IBus.normalize(res[2])/100) ) ) )
                                pca.channels[frontRight_forward].duty_cycle = motorSpeed
                                pca.channels[frontLeft_forward].duty_cycle = motorSpeed
                                pca.channels[rearRight_forward].duty_cycle = motorSpeed
                                pca.channels[rearLeft_forward].duty_cycle = motorSpeed
                                pca.channels[frontRight_backward].duty_cycle = 0
                                pca.channels[frontLeft_backward].duty_cycle = 0
                                pca.channels[rearRight_backward].duty_cycle = 0
                                pca.channels[rearLeft_backward].duty_cycle = 0
                        #Backward control
                        elif(int(maxSpeed * (IBus.normalize(res[2])/100)) < -0.5):
                                #go backward
                                motorSpeed = abs(int(maxSpeed * ((poten/100) * (IBus.normalize(res[2])/100) ) ) )
                                pca.channels[frontRight_backward].duty_cycle = motorSpeed
                                pca.channels[frontLeft_backward].duty_cycle = motorSpeed
                                pca.channels[rearRight_backward].duty_cycle = motorSpeed
                                pca.channels[rearLeft_backward].duty_cycle = motorSpeed
                                pca.channels[frontRight_forward].duty_cycle = 0
                                pca.channels[frontLeft_forward].duty_cycle = 0
                                pca.channels[rearRight_forward].duty_cycle = 0
                                pca.channels[rearLeft_forward].duty_cycle = 0
                        #stay still
                        else:
                                pca.channels[frontRight_forward].duty_cycle = 0
                                pca.channels[frontLeft_forward].duty_cycle = 0
                                pca.channels[rearRight_forward].duty_cycle = 0
                                pca.channels[rearLeft_forward].duty_cycle = 0
                                pca.channels[frontRight_backward].duty_cycle = 0
                                pca.channels[frontLeft_backward].duty_cycle = 0
                                pca.channels[rearRight_backward].duty_cycle = 0
                                pca.channels[rearLeft_backward].duty_cycle = 0

                        #steering
                        if(IBus.normalize(res[1]) > 0.5):
                                #turn right
                                pca.channels[steering_forward].duty_cycle = abs(int((maxSpeed * (IBus.normalize(res[1])/100)))) #half a percent of the max 15 amps the battery can give
                        elif(IBus.normalize(res[1]) < -0.5):
                                #turn left
                                pca.channels[steering_backward].duty_cycle = abs(int((maxSpeed * (IBus.normalize(res[1])/100)))) #half a percent of the max 15 amps the battery can give
                        else:
                                #stand still
                                pca.channels[steering_forward].duty_cycle = 0
                                pca.channels[steering_backward].duty_cycle = 0

            else:
                stopFunction()

def stopFunction():

        GPIO.output(FR_en, 1)

        GPIO.output(FL_en, 1)

        GPIO.output(RR_en, 1)

        GPIO.output(RL_en, 1)

        #stop all motors (arresto momentum)
        pca.channels[frontRight_forward].duty_cycle = 0
        pca.channels[frontLeft_forward].duty_cycle = 0
        pca.channels[rearRight_forward].duty_cycle = 0
        pca.channels[rearLeft_forward].duty_cycle = 0
        pca.channels[frontRight_backward].duty_cycle = 0
        pca.channels[frontLeft_backward].duty_cycle = 0
        pca.channels[rearRight_backward].duty_cycle = 0
        pca.channels[rearLeft_backward].duty_cycle = 0

        time.sleep(10)

        sys.exit(1)

if __name__ == "__main__":
        manager = multiprocessing.Manager()
        mutex = manager.Lock()
        p1 = multiprocessing.Process(target=status, args=(mutex,))
        p2 = multiprocessing.Process(target=main, args=(mutex,))

        p1.start()
        p2.start()

        p1.join()
        p2.join()
