import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) #set pin GPIO 17 to be input pin and set initial value to be pulled low(off) 
GPIO.setup(21, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(15,GPIO.OUT)
GPIO.setup(14,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
GPIO.setup(25,GPIO.OUT)

while True:
    if GPIO.input(20)==GPIO.LOW and GPIO.input(21)==GPIO.LOW:     #if the state for switch pin is pressed or GPIO.LOW then limit switch (ls)=1 
        ls=1                         #declared value of limit switch
        print("Switch pushed")       #print statement saying which state it is in
        time.sleep(.1)
    else:
        ls=0                         #limit switch is value 0 stating the switch is not pressed
        print("Switch off")          #print statement saying the state it is in
        GPIO.output(12,0)            #is relay 1 that makes the base arms retract for arm 1
        GPIO.output(15,0)            #is relay 2 that makes the base arm retract for arm 2
        time.sleep(.1)
        GPIO.output(25,1)             #is relay 
        GPIO.output(14,1)
        
    if ls:      #drone and ls:                  #ls = limit switch
        GPIO.output(25,0)
        GPIO.output(14,0)
        time.sleep(.1)
        GPIO.output(12,1)
        GPIO.output(15,1)
