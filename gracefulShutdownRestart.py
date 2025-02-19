#!//home/edd/.venv/bin/python3
from gpiozero import Button
import os
import psutil
from signal import pause

shutdownPin = 20
restartPin = 21

def shutdown():
    os.system("poweroff")
    #print("shutdown button activated")

def restart():
    os.system("reboot")
    print("restart button activated")

shutdownBtn = Button(shutdownPin, hold_time=1.5)
shutdownBtn.when_held = shutdown

restartBtn = Button(restartPin, hold_time=1.5)
restartBtn.when_held = restart

pause()
