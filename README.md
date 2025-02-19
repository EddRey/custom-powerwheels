# custom-powerwheels
Python script and tools for controlling motors connected to the four wheels of a child's ride on toy.<br/>
Made to work with raspberry Pi receiving signals from physical buttons and switches<br/><br/>
Necessary peripherals:
- Raspberry Pi 4 (Im sure any Raspberry Pi will work but I had a spare 4 that I used)
- An IBus capable controller with reciever.<br/>
- BTS7960 for controlling motors<br/>
- PCF8575 for more GPIO input<br/>
- PCA9685 for PWM signals<br/>
- ADS1115 for analog-to-digital for potentiometer<br/>
- A Speaker for status and fun horn honk/beep beep sounds<br/>
- A sacrificial Ride on toy. The one I used had All Wheel Drive so reused the 2xRS550 and 2xRS775 motors that it came with<br/>
- 18 gauge wire for connecting low amperage components such as PCB to PCB<br/>
- 12-14 gauge wire for connected higher amperage such as BTS7960 to power supply<br/>
