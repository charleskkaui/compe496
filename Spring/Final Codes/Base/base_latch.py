import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time

#GPIO Initializations
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off)
GPIO.setup(16,GPIO.OUT)
GPIO.setup(4,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
GPIO.setup(13,GPIO.OUT)

while True:
    if GPIO.input(20)==GPIO.LOW and GPIO.input(28)==GPIO.LOW:     #if the state for switch pin is pressed or GPIO.LOW then limit switch (ls)=1 
        ls=1                         #declared value of limit switch        
        
    if ls:      #drone and ls:                  #ls = limit switch
        print("Switch pushed")       #print statement saying which state it is in
        time.sleep(.1)
        GPIO.output(4,0)
        GPIO.output(12,0)
        time.sleep(.1)
        GPIO.output(16,1)
        GPIO.output(13,1)